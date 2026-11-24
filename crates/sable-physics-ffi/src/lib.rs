//! Personal-port native boundary. Solver/source pins and global ownership remain
//! in sable-physics; this crate only translates validated C-layout messages.
use sable_bridge::abi::{self, *};
use sable_physics::{
    BodyHandle, BodyMass, ContactPolicy, ContactRequest, ContactResponse, ContactWorld, Engine,
    JointHandle, Material, MaterialHandle, SceneHandle,
};
use std::cell::{Cell as LocalCell, RefCell};
use std::collections::{BTreeMap, BTreeSet};
use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering};

static NEXT: AtomicU64 = AtomicU64::new(1);
static ACTIVE_OWNER: AtomicU64 = AtomicU64::new(0);
static PANICKED: AtomicBool = AtomicBool::new(false);
thread_local! {
    static OWNER: RefCell<Option<Owner>> = const { RefCell::new(None) };
    static CONTACT_WORLD: LocalCell<Option<(u64, usize)>> = const { LocalCell::new(None) };
}
fn token() -> Result<u64, u32> {
    NEXT.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |v| v.checked_add(1))
        .map_err(|_| status::INVALID)
}
fn mapped(error: sable_physics::Error) -> u32 {
    match error {
        sable_physics::Error::ProcessBusy => status::BUSY,
        sable_physics::Error::ReentrantOperation => status::REENTRANT,
        sable_physics::Error::StaleScene
        | sable_physics::Error::StaleBody
        | sable_physics::Error::StaleJoint => status::STALE,
        sable_physics::Error::WrongScene => status::WRONG_SCENE,
        sable_physics::Error::AlreadyExists => status::EXISTS,
        sable_physics::Error::InvalidInput => status::INVALID,
    }
}
struct Owner {
    token: u64,
    engine: Engine,
    scenes: BTreeMap<u64, SceneHandle>,
    bodies: BTreeMap<u64, (u64, BodyHandle)>,
    materials: BTreeMap<u64, MaterialHandle>,
    joints: BTreeMap<u64, (u64, JointHandle)>,
    callbacks: Vec<Arc<Policy>>,
    collisions: BTreeMap<u64, Vec<f64>>,
}
impl Drop for Owner {
    fn drop(&mut self) {
        for callback in &self.callbacks {
            callback.active.store(false, Ordering::Release);
        }
        ACTIVE_OWNER.store(0, Ordering::Release);
    }
}
impl Owner {
    fn scene(&self, id: u64) -> Result<SceneHandle, u32> {
        self.scenes.get(&id).copied().ok_or(status::STALE)
    }
    fn body(&self, id: u64) -> Result<BodyHandle, u32> {
        self.bodies.get(&id).map(|(_, h)| *h).ok_or(status::STALE)
    }
    fn material(&self, id: u64) -> Result<MaterialHandle, u32> {
        self.materials.get(&id).copied().ok_or(status::STALE)
    }
    fn optional_body(&self, id: u64) -> Result<Option<BodyHandle>, u32> {
        if id == 0 {
            Ok(None)
        } else {
            self.body(id).map(Some)
        }
    }
    fn callback_error(&self) -> Result<(), u32> {
        if self
            .callbacks
            .iter()
            .any(|p| p.failure.load(Ordering::Acquire) != 0)
        {
            Err(status::CALLBACK)
        } else {
            Ok(())
        }
    }
    fn execute(&mut self, request: &Request, response: &mut Response) -> Result<(), u32> {
        let q = request;
        if self
            .callbacks
            .iter()
            .any(|policy| policy.failure.load(Ordering::Acquire) != 0)
            && !matches!(
                q.operation,
                op::POSE | op::VELOCITY | op::JOINT_SAMPLE | op::COPY_COLLISIONS
            )
        {
            return Err(status::CALLBACK);
        }
        match q.operation {
            op::SCENE => {
                let id = token()?;
                let handle = self
                    .engine
                    .create_scene(q.values[..3].try_into().unwrap(), q.values[3])
                    .map_err(mapped)?;
                self.scenes.insert(id, handle);
                response.handle = id;
            }
            op::DISPOSE_SCENE => {
                let h = self.scene(q.scene)?;
                self.engine.dispose_scene(h).map_err(mapped)?;
                self.scenes.remove(&q.scene);
                self.bodies.retain(|_, (scene, _)| *scene != q.scene);
                self.joints.retain(|_, (scene, _)| *scene != q.scene);
                self.collisions.remove(&q.scene);
            }
            op::CONFIGURE => self
                .engine
                .configure(q.coord, [q.values[0], q.values[1]], q.id)
                .map_err(mapped)?,
            op::MATERIAL => {
                let id = token()?;
                let boxes = read_slice::<[f64; 6]>(q.data, q.length, abi::MAX_BOXES)?.to_vec();
                let policy = q.contact.map(|function| {
                    Arc::new(Policy {
                        function,
                        context: q.context,
                        active: AtomicBool::new(true),
                        failure: AtomicU32::new(0),
                    })
                });
                let material = self
                    .engine
                    .register_material(Material {
                        friction: q.values[0],
                        volume: q.values[1],
                        restitution: q.values[2],
                        fluid: q.flags & 1 != 0,
                        dynamic: q.flags & 2 != 0,
                        boxes,
                        contact: policy
                            .as_ref()
                            .map(|p| Arc::clone(p) as Arc<dyn ContactPolicy>),
                    })
                    .map_err(mapped)?;
                if let Some(policy) = policy {
                    self.callbacks.push(policy);
                }
                self.materials.insert(id, material);
                response.handle = id;
                response.count = self.engine.material_index(material) as u64;
            }
            op::CLEAR_BOXES => self
                .engine
                .clear_material_boxes(self.material(q.object)?)
                .map_err(mapped)?,
            op::ADD_BOX => self
                .engine
                .add_material_box(self.material(q.object)?, q.values[..6].try_into().unwrap())
                .map_err(mapped)?,
            op::BOX => {
                let s = self.scene(q.scene)?;
                let id = token()?;
                let h = self
                    .engine
                    .create_box(
                        s,
                        q.id,
                        q.values[0],
                        q.values[1..4].try_into().unwrap(),
                        q.values[4..11].try_into().unwrap(),
                    )
                    .map_err(mapped)?;
                self.bodies.insert(id, (q.scene, h));
                response.handle = id;
            }
            op::SUBLEVEL => {
                let id = token()?;
                let h = self
                    .engine
                    .create_sublevel(
                        self.scene(q.scene)?,
                        q.id,
                        q.values[13..20].try_into().unwrap(),
                        q.bounds,
                        BodyMass {
                            mass: q.values[0],
                            center: q.values[1..4].try_into().unwrap(),
                            inertia: q.values[4..13].try_into().unwrap(),
                        },
                    )
                    .map_err(mapped)?;
                self.bodies.insert(id, (q.scene, h));
                response.handle = id;
            }
            op::POSE => response.values[..7]
                .copy_from_slice(&self.engine.pose(self.body(q.object)?).map_err(mapped)?),
            op::VELOCITY => response.values[..6]
                .copy_from_slice(&self.engine.velocity(self.body(q.object)?).map_err(mapped)?),
            op::IMPULSE | op::ADD_VELOCITY => {
                let body = self.body(q.object)?;
                let linear = q.values[..3].try_into().unwrap();
                let angular = q.values[3..6].try_into().unwrap();
                if q.operation == op::IMPULSE {
                    self.engine
                        .impulse(body, linear, angular, q.flags != 0)
                        .map_err(mapped)?;
                } else {
                    self.engine
                        .add_velocity(body, linear, angular, q.flags != 0)
                        .map_err(mapped)?;
                }
            }
            op::WAKE => self.engine.wake(self.body(q.object)?).map_err(mapped)?,
            op::TICK => {
                self.engine.tick(self.scene(q.scene)?).map_err(mapped)?;
                self.callback_error()?;
            }
            op::STEP => {
                self.engine
                    .step(self.scene(q.scene)?, q.values[0])
                    .map_err(mapped)?;
                self.callback_error()?;
            }
            op::UPLOAD => {
                let scene = self.scene(q.scene)?;
                let body = self.optional_body(q.object)?;
                let input = read_slice::<abi::Cell>(q.data, q.length, abi::MAX_CELLS)?;
                let mut seen = BTreeSet::new();
                let mut cells = Vec::with_capacity(input.len());
                for cell in input {
                    if cell.reserved != 0 || !seen.insert(cell.position) {
                        return Err(status::INVALID);
                    }
                    cells.push((cell.position, self.material(cell.material)?, cell.state));
                }
                self.engine
                    .upload_chunk(scene, q.coord, q.flags != 0, body, cells)
                    .map_err(mapped)?;
            }
            op::REMOVE_CHUNK => self
                .engine
                .remove_chunk(self.scene(q.scene)?, q.coord, q.flags != 0)
                .map_err(mapped)?,
            op::REMOVE_BODY => {
                self.engine
                    .remove_body(self.body(q.object)?)
                    .map_err(mapped)?;
                self.bodies.remove(&q.object);
            }
            op::FIXED_JOINT => {
                let id = token()?;
                let joint = self
                    .engine
                    .fixed_joint(
                        self.scene(q.scene)?,
                        self.optional_body(q.object)?,
                        self.optional_body(q.other)?,
                        q.values[..6].try_into().unwrap(),
                        q.values[6..10].try_into().unwrap(),
                    )
                    .map_err(mapped)?;
                self.joints.insert(id, (q.scene, joint));
                response.handle = id;
            }
            op::JOINT_SAMPLE => {
                let (_, joint) = self.joints.get(&q.object).copied().ok_or(status::STALE)?;
                let (valid, values) = self.engine.joint_sample(joint).map_err(mapped)?;
                response.count = u64::from(valid);
                response.values[..6].copy_from_slice(&values);
            }
            op::REMOVE_JOINT => {
                let (_, joint) = self.joints.get(&q.object).copied().ok_or(status::STALE)?;
                self.engine.remove_joint(joint).map_err(mapped)?;
                // Keep the token for the accepted invalid-after-remove sample.
            }
            op::CAPTURE_COLLISIONS => {
                let values = self
                    .engine
                    .collisions(self.scene(q.scene)?)
                    .map_err(mapped)?;
                response.count = values.len() as u64;
                self.collisions.insert(q.scene, values);
            }
            op::COPY_COLLISIONS => {
                self.scene(q.scene)?;
                let values = self.collisions.get(&q.scene).ok_or(status::INVALID)?;
                if (response.capacity as usize) < values.len() {
                    return Err(status::BUFFER);
                }
                if !values.is_empty() {
                    if !valid_mut(response.data) {
                        return Err(status::INVALID);
                    }
                    // SAFETY: the ABI caller promises capacity writable f64 elements.
                    unsafe {
                        std::ptr::copy_nonoverlapping(values.as_ptr(), response.data, values.len());
                    }
                }
                response.written = values.len() as u32;
                response.count = values.len() as u64;
            }
            _ => return Err(status::INVALID),
        }
        Ok(())
    }
}

fn valid_mut<T>(p: *mut T) -> bool {
    !p.is_null() && (p as usize).is_multiple_of(std::mem::align_of::<T>())
}
fn read_slice<'a, T>(
    pointer: *const std::ffi::c_void,
    length: u32,
    limit: usize,
) -> Result<&'a [T], u32> {
    let length = length as usize;
    if length > limit {
        return Err(status::INVALID);
    }
    if length == 0 {
        return Ok(&[]);
    }
    if pointer.is_null() || !(pointer as usize).is_multiple_of(std::mem::align_of::<T>()) {
        return Err(status::INVALID);
    }
    // SAFETY: non-null/alignment/bounded length checked; readable allocation is
    // the unsafe caller's ABI obligation for this one call.
    Ok(unsafe { std::slice::from_raw_parts(pointer.cast(), length) })
}
fn validate_request(q: &Request) -> Result<(), u32> {
    if !(op::OPEN..=op::COPY_COLLISIONS).contains(&q.operation) || q.reserved != 0 {
        return Err(status::INVALID);
    }
    let flags = match q.operation {
        op::MATERIAL => 3,
        op::IMPULSE | op::ADD_VELOCITY | op::UPLOAD | op::REMOVE_CHUNK => 1,
        _ => 0,
    };
    if q.flags & !flags != 0 {
        return Err(status::INVALID);
    }
    let count = match q.operation {
        op::SCENE => 4,
        op::CONFIGURE => 2,
        op::MATERIAL => 3,
        op::ADD_BOX | op::IMPULSE | op::ADD_VELOCITY => 6,
        op::BOX => 11,
        op::SUBLEVEL => 20,
        op::STEP => 1,
        op::FIXED_JOINT => 10,
        _ => 0,
    };
    if q.values[count..].iter().any(|v| v.to_bits() != 0) {
        return Err(status::INVALID);
    }
    if !matches!(q.operation, op::MATERIAL | op::UPLOAD) && (!q.data.is_null() || q.length != 0) {
        return Err(status::INVALID);
    }
    if q.operation != op::MATERIAL && q.contact.is_some() || q.contact.is_none() && q.context != 0 {
        return Err(status::INVALID);
    }
    if q.operation != op::SUBLEVEL && q.bounds != [0; 6] {
        return Err(status::INVALID);
    }
    if !matches!(q.operation, op::CONFIGURE | op::UPLOAD | op::REMOVE_CHUNK) && q.coord != [0; 3] {
        return Err(status::INVALID);
    }
    if !matches!(q.operation, op::CONFIGURE | op::BOX | op::SUBLEVEL) && q.id != 0 {
        return Err(status::INVALID);
    }
    if q.operation == op::OPEN && q.owner != 0 {
        return Err(status::INVALID);
    }
    if !matches!(
        q.operation,
        op::DISPOSE_SCENE
            | op::BOX
            | op::SUBLEVEL
            | op::TICK
            | op::STEP
            | op::UPLOAD
            | op::REMOVE_CHUNK
            | op::FIXED_JOINT
            | op::CAPTURE_COLLISIONS
            | op::COPY_COLLISIONS
    ) && q.scene != 0
    {
        return Err(status::INVALID);
    }
    if !matches!(
        q.operation,
        op::CLEAR_BOXES
            | op::ADD_BOX
            | op::POSE
            | op::VELOCITY
            | op::IMPULSE
            | op::ADD_VELOCITY
            | op::WAKE
            | op::UPLOAD
            | op::REMOVE_BODY
            | op::FIXED_JOINT
            | op::JOINT_SAMPLE
            | op::REMOVE_JOINT
    ) && q.object != 0
    {
        return Err(status::INVALID);
    }
    if q.operation != op::FIXED_JOINT && q.other != 0 {
        return Err(status::INVALID);
    }
    Ok(())
}
fn execute_inner(input: *const Request, output: *mut Response) -> Result<(), u32> {
    if !valid_mut(input.cast_mut()) || !valid_mut(output) {
        return Err(status::INVALID);
    }
    // Read the common header before any larger layout or mutation.
    let input_header = unsafe { input.cast::<Header>().read() };
    let output_header = unsafe { output.cast::<Header>().read() };
    if !input_header.valid::<Request>() || !output_header.valid::<Response>() {
        return Err(status::VERSION);
    }
    let request = unsafe { input.read() };
    validate_request(&request)?;
    if PANICKED.load(Ordering::Acquire) && request.operation != op::CLOSE {
        return Err(status::PANIC);
    }
    let response = unsafe { &mut *output };
    if request.operation != op::COPY_COLLISIONS
        && (!response.data.is_null() || response.capacity != 0)
    {
        return Err(status::INVALID);
    }
    OWNER.with(|slot| {
        let mut slot = slot.try_borrow_mut().map_err(|_| status::REENTRANT)?;
        if request.operation == op::OPEN {
            if slot.is_some() || ACTIVE_OWNER.load(Ordering::Acquire) != 0 {
                return Err(status::BUSY);
            }
            let owner = token()?;
            let engine = Engine::try_new().map_err(mapped)?;
            *slot = Some(Owner {
                token: owner,
                engine,
                scenes: BTreeMap::new(),
                bodies: BTreeMap::new(),
                materials: BTreeMap::new(),
                joints: BTreeMap::new(),
                callbacks: Vec::new(),
                collisions: BTreeMap::new(),
            });
            ACTIVE_OWNER.store(owner, Ordering::Release);
            response.handle = owner;
            return Ok(());
        }
        let current = ACTIVE_OWNER.load(Ordering::Acquire);
        let owner = slot
            .as_mut()
            .ok_or(if current == request.owner && current != 0 {
                status::THREAD
            } else {
                status::STALE
            })?;
        if owner.token != request.owner {
            return Err(status::STALE);
        }
        if request.operation == op::CLOSE {
            slot.take();
            return Ok(());
        }
        owner.execute(&request, response)
    })
}
unsafe extern "C" fn execute(input: *const Request, output: *mut Response) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| execute_inner(input, output))) {
        Ok(Ok(())) => status::OK,
        Ok(Err(status)) => status,
        Err(_) => {
            PANICKED.store(true, Ordering::Release);
            status::PANIC
        }
    }
}

struct ScopedWorld<'a> {
    world: &'a dyn ContactWorld,
}
#[derive(Debug)]
struct Policy {
    function: ContactFn,
    context: u64,
    active: AtomicBool,
    failure: AtomicU32,
}
impl ContactPolicy for Policy {
    fn on_contact(&self, input: ContactRequest, world: &dyn ContactWorld) -> ContactResponse {
        if !self.active.load(Ordering::Acquire) {
            return ContactResponse {
                reject: self.failure.load(Ordering::Acquire) != 0,
                ..Default::default()
            };
        }
        let Ok(id) = token() else {
            self.failure.store(status::INVALID, Ordering::Release);
            return ContactResponse {
                reject: true,
                ..Default::default()
            };
        };
        let scoped = ScopedWorld { world };
        CONTACT_WORLD
            .with(|active| active.set(Some((id, (&scoped as *const ScopedWorld<'_>) as usize))));
        struct Reset;
        impl Drop for Reset {
            fn drop(&mut self) {
                CONTACT_WORLD.with(|active| active.set(None));
            }
        }
        let _reset = Reset;
        let request = Contact {
            header: Header::of::<Contact>(),
            block: input.block,
            other: input.other,
            point: input.point,
            speed: input.speed,
            has_other: u32::from(input.has_other),
            reserved: 0,
            world: id,
        };
        let mut response = ContactReply::default();
        // SAFETY: the registered foreign callback obeys ContactFn's C ABI and
        // receives only call-scoped pointers and the validated service nonce.
        let result =
            unsafe { (self.function)(self.context, &request, world_service, &mut response) };
        if result != status::OK
            || !response.header.valid::<ContactReply>()
            || response.reserved != 0
            || response.reject > 1
            || !response
                .tangent
                .iter()
                .all(|v| v.is_finite() && (*v as f32).is_finite())
        {
            self.active.store(false, Ordering::Release);
            self.failure.store(status::CALLBACK, Ordering::Release);
            return ContactResponse {
                reject: true,
                ..Default::default()
            };
        }
        ContactResponse {
            tangent: response.tangent,
            reject: response.reject != 0,
        }
    }
}
fn world_inner(input: *const WorldRequest, output: *mut WorldResponse) -> Result<(), u32> {
    if !valid_mut(input.cast_mut()) || !valid_mut(output) {
        return Err(status::INVALID);
    }
    if !unsafe { input.cast::<Header>().read() }.valid::<WorldRequest>()
        || !unsafe { output.cast::<Header>().read() }.valid::<WorldResponse>()
    {
        return Err(status::VERSION);
    }
    let request = unsafe { &*input };
    let output = unsafe { &mut *output };
    CONTACT_WORLD.with(|active| {
        let (id, address) = active.get().ok_or(status::STALE)?;
        if id != request.world {
            return Err(status::STALE);
        }
        // SAFETY: the nonce is active only on this exact callback thread while
        // ScopedWorld remains on its stack; it cannot be retained across calls.
        let context = unsafe { &*(address as *const ScopedWorld<'_>) };
        let mut result = WorldResponse::default();
        match request.operation {
            1 => {
                if let Some((material, state)) = context.world.block(request.position) {
                    result.present = 1;
                    result.material = material;
                    result.state = u32::from(state);
                }
            }
            2 => context
                .world
                .remove_block(request.position)
                .map_err(|_| status::INVALID)?,
            _ => return Err(status::INVALID),
        }
        *output = result;
        Ok(())
    })
}
unsafe extern "C" fn world_service(input: *const WorldRequest, output: *mut WorldResponse) -> u32 {
    match catch_unwind(AssertUnwindSafe(|| world_inner(input, output))) {
        Ok(Ok(())) => status::OK,
        Ok(Err(s)) => s,
        Err(_) => {
            PANICKED.store(true, Ordering::Release);
            status::PANIC
        }
    }
}

/// Obtain the sole versioned function table. No engine is created by discovery.
///
/// # Safety
/// `output` must point to writable storage for one `Api`; all subsequently used
/// request/response pointers must obey their documented one-call storage bounds.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sable_physics_get_api(version: u32, size: u32, output: *mut Api) -> u32 {
    if version != ABI_VERSION || size != std::mem::size_of::<Api>() as u32 {
        return status::VERSION;
    }
    if !valid_mut(output) {
        return status::INVALID;
    }
    unsafe {
        output.write(Api {
            header: Header::of::<Api>(),
            execute,
        });
    }
    status::OK
}
