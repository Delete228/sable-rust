//! SDK-compiler client for the separately built accepted-nightly native library.
//! This crate has no dependency on sable-core, sable-physics, Rapier or the SDK.
//! One library is pinned/mapped for process lifetime: its process-global solver
//! materials and Rayon workers must never outlive unloaded machine code.
pub mod abi;
use abi::{op, status, *};
use std::collections::BTreeMap;
use std::ffi::{CStr, CString, c_char, c_void};
use std::marker::PhantomData;
use std::path::{Path, PathBuf};
use std::rc::Rc;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Native(u32),
    Load(String),
    DifferentLibrary,
    ProcessBusy,
    InvalidInput,
}
pub type Result<T> = std::result::Result<T, Error>;
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Scene(pub u64);
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Body(pub u64);
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MaterialId {
    pub handle: u64,
    pub index: u64,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Joint(pub u64);
#[derive(Debug, Clone, Copy)]
pub struct Mass {
    pub mass: f64,
    pub center: [f64; 3],
    pub inertia: [f64; 9],
}
#[derive(Debug, Clone)]
pub struct Material {
    pub friction: f64,
    pub volume: f64,
    pub restitution: f64,
    pub fluid: bool,
    pub dynamic: bool,
    pub boxes: Vec<[f64; 6]>,
    pub contact: Option<Arc<dyn ContactPolicy>>,
}
pub trait ContactPolicy: std::fmt::Debug + Send + Sync {
    fn on_contact(&self, input: Contact, world: &ContactWorld<'_>) -> ContactReply;
}
/// Borrowed only for the synchronous callback. It cannot cross threads or escape
/// the callback; the native side separately checks a thread-local service nonce.
pub struct ContactWorld<'a> {
    nonce: u64,
    service: WorldFn,
    _scope: PhantomData<(&'a (), Rc<()>)>,
}
impl ContactWorld<'_> {
    fn call(&self, operation: u32, position: [i32; 3]) -> Result<WorldResponse> {
        let request = WorldRequest {
            header: Header::of::<WorldRequest>(),
            world: self.nonce,
            operation,
            position,
        };
        let mut response = WorldResponse::default();
        let status = unsafe { (self.service)(&request, &mut response) };
        checked(status)?;
        if !response.header.valid::<WorldResponse>() || response.present > 1 || response.state > 4 {
            return Err(Error::Native(status::VERSION));
        }
        Ok(response)
    }
    /// Low-level nonce and C service entry. Values may be retained for stale-token
    /// tests, but every invocation remains unsafe and active only in this callback.
    pub fn protocol_service(&self) -> (u64, WorldFn) {
        (self.nonce, self.service)
    }
    pub fn block(&self, position: [i32; 3]) -> Result<Option<(u32, u8)>> {
        let response = self.call(1, position)?;
        Ok((response.present != 0).then_some((response.material, response.state as u8)))
    }
    pub fn remove_block(&self, position: [i32; 3]) -> Result<()> {
        self.call(2, position).map(|_| ())
    }
}
static CALLBACKS: Mutex<BTreeMap<u64, Arc<dyn ContactPolicy>>> = Mutex::new(BTreeMap::new());
static NEXT_CALLBACK: AtomicU64 = AtomicU64::new(1);
static INSTANCE: AtomicBool = AtomicBool::new(false);
struct Loaded {
    path: PathBuf,
    api: Api,
    _mapping: usize,
}
static LIBRARY: Mutex<Option<Loaded>> = Mutex::new(None);
#[cfg_attr(target_os = "linux", link(name = "dl"))]
unsafe extern "C" {
    fn dlopen(path: *const c_char, flags: i32) -> *mut c_void;
    fn dlsym(handle: *mut c_void, name: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> i32;
    fn dlerror() -> *const c_char;
}
fn loader_error() -> String {
    let error = unsafe { dlerror() };
    if error.is_null() {
        "dynamic loader refused library".into()
    } else {
        unsafe { CStr::from_ptr(error) }
            .to_string_lossy()
            .into_owned()
    }
}
fn load(path: &Path) -> Result<Api> {
    use std::os::unix::ffi::OsStrExt;
    let path = path
        .canonicalize()
        .map_err(|e| Error::Load(e.to_string()))?;
    let mut slot = LIBRARY
        .lock()
        .map_err(|_| Error::Load("loader lock poisoned".into()))?;
    if let Some(loaded) = slot.as_ref() {
        return if loaded.path == path {
            Ok(loaded.api)
        } else {
            Err(Error::DifferentLibrary)
        };
    }
    let name = CString::new(path.as_os_str().as_bytes()).map_err(|_| Error::InvalidInput)?;
    // Darwin defines RTLD_LOCAL as 4; Linux uses 0. Keep the private nightly
    // runtime's symbols out of the SDK/native-mod global lookup scope.
    let flags = if cfg!(target_os = "macos") { 2 | 4 } else { 2 };
    let handle = unsafe { dlopen(name.as_ptr(), flags) };
    if handle.is_null() {
        return Err(Error::Load(loader_error()));
    }
    let entry = unsafe { dlsym(handle, c"sable_physics_get_api".as_ptr()) };
    if entry.is_null() {
        let error = loader_error();
        unsafe {
            dlclose(handle);
        }
        return Err(Error::Load(error));
    }
    // SAFETY: exact C symbol, explicit version and output size are negotiated
    // before using any table member; the library is an explicitly trusted input.
    let get_api: GetApiFn = unsafe { std::mem::transmute(entry) };
    let mut output = std::mem::MaybeUninit::<Api>::uninit();
    let result = unsafe {
        get_api(
            ABI_VERSION,
            std::mem::size_of::<Api>() as u32,
            output.as_mut_ptr(),
        )
    };
    if result != status::OK {
        unsafe {
            dlclose(handle);
        }
        return Err(Error::Native(result));
    }
    let api = unsafe { output.assume_init() };
    if !api.header.valid::<Api>() {
        unsafe {
            dlclose(handle);
        }
        return Err(Error::Native(status::VERSION));
    }
    *slot = Some(Loaded {
        path,
        api,
        _mapping: handle as usize,
    });
    Ok(api)
}
fn next_callback() -> Result<u64> {
    let mut current = NEXT_CALLBACK.load(Ordering::Relaxed);
    loop {
        let next = current.checked_add(1).ok_or(Error::InvalidInput)?;
        match NEXT_CALLBACK.compare_exchange_weak(
            current,
            next,
            Ordering::Relaxed,
            Ordering::Relaxed,
        ) {
            Ok(_) => return Ok(current),
            Err(actual) => current = actual,
        }
    }
}
fn checked(code: u32) -> Result<()> {
    if code == status::OK {
        Ok(())
    } else {
        Err(Error::Native(code))
    }
}
unsafe extern "C" fn contact(
    context: u64,
    input: *const Contact,
    service: WorldFn,
    output: *mut ContactReply,
) -> u32 {
    let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
        if input.is_null() || output.is_null() {
            return Err(status::INVALID);
        }
        let input = unsafe { *input };
        if !input.header.valid::<Contact>() {
            return Err(status::VERSION);
        }
        let policy = CALLBACKS
            .lock()
            .map_err(|_| status::CALLBACK)?
            .get(&context)
            .cloned()
            .ok_or(status::CALLBACK)?;
        let world = ContactWorld {
            nonce: input.world,
            service,
            _scope: PhantomData,
        };
        let reply = policy.on_contact(input, &world);
        unsafe {
            output.write(reply);
        }
        Ok(())
    }));
    match result {
        Ok(Ok(())) => status::OK,
        Ok(Err(s)) => s,
        Err(_) => status::CALLBACK,
    }
}

/// Serialized owner, intentionally neither Send nor Sync. Closing destroys the
/// actual Engine on this thread and disables its persistent material callbacks.
pub struct Bridge {
    api: Api,
    owner: u64,
    callbacks: Vec<u64>,
    _thread: PhantomData<Rc<()>>,
}
impl Bridge {
    pub fn open(path: impl AsRef<Path>) -> Result<Self> {
        if INSTANCE.swap(true, Ordering::AcqRel) {
            return Err(Error::ProcessBusy);
        }
        let attempt = (|| {
            let api = load(path.as_ref())?;
            let request = Request::new(op::OPEN);
            let mut response = Response::default();
            checked(unsafe { (api.execute)(&request, &mut response) })?;
            Ok(Self {
                api,
                owner: response.handle,
                callbacks: Vec::new(),
                _thread: PhantomData,
            })
        })();
        if attempt.is_err() {
            INSTANCE.store(false, Ordering::Release);
        }
        attempt
    }
    fn call(&mut self, mut request: Request) -> Result<Response> {
        request.owner = self.owner;
        let mut response = Response::default();
        checked(unsafe { (self.api.execute)(&request, &mut response) })?;
        if !response.header.valid::<Response>() {
            return Err(Error::Native(status::VERSION));
        }
        Ok(response)
    }
    /// Raw calls are useful for protocol tests and language bindings.
    ///
    /// # Safety
    /// Pointer storage must be valid, aligned and nonoverlapping for the stated
    /// length. Callback pointers must be valid C functions which never unwind or
    /// retain call-scoped data; callers must not bypass exclusive Engine ownership.
    pub unsafe fn raw_call(&self, request: &Request, response: &mut Response) -> u32 {
        unsafe { (self.api.execute)(request, response) }
    }
    pub fn owner_token(&self) -> u64 {
        self.owner
    }
    /// Low-level C entry; invoking it remains unsafe and requires the ABI contract.
    pub fn protocol_entry(&self) -> ExecuteFn {
        self.api.execute
    }
    pub fn close(&mut self) -> Result<()> {
        if self.owner == 0 {
            return Ok(());
        }
        self.call(Request::new(op::CLOSE))?;
        self.owner = 0;
        if let Ok(mut policies) = CALLBACKS.lock() {
            for id in self.callbacks.drain(..) {
                policies.remove(&id);
            }
        }
        INSTANCE.store(false, Ordering::Release);
        Ok(())
    }
    pub fn scene(&mut self, gravity: [f64; 3], drag: f64) -> Result<Scene> {
        let mut q = Request::new(op::SCENE);
        q.values[..3].copy_from_slice(&gravity);
        q.values[3] = drag;
        self.call(q).map(|r| Scene(r.handle))
    }
    pub fn dispose_scene(&mut self, scene: Scene) -> Result<()> {
        let mut q = Request::new(op::DISPOSE_SCENE);
        q.scene = scene.0;
        self.call(q).map(|_| ())
    }
    pub fn configure(
        &mut self,
        iterations: [i32; 3],
        frequency: [f64; 2],
        minimum_island: i32,
    ) -> Result<()> {
        let mut q = Request::new(op::CONFIGURE);
        q.coord = iterations;
        q.values[..2].copy_from_slice(&frequency);
        q.id = minimum_island;
        self.call(q).map(|_| ())
    }
    pub fn material(&mut self, material: Material) -> Result<MaterialId> {
        if material.boxes.len() > MAX_BOXES {
            return Err(Error::InvalidInput);
        }
        let mut q = Request::new(op::MATERIAL);
        q.values[..3].copy_from_slice(&[material.friction, material.volume, material.restitution]);
        q.flags = u32::from(material.fluid) | (u32::from(material.dynamic) << 1);
        q.data = material.boxes.as_ptr().cast();
        q.length = material.boxes.len() as u32;
        let callback = if let Some(policy) = material.contact {
            let id = next_callback()?;
            CALLBACKS
                .lock()
                .map_err(|_| Error::InvalidInput)?
                .insert(id, policy);
            q.contact = Some(contact);
            q.context = id;
            Some(id)
        } else {
            None
        };
        let result = self.call(q).map(|r| MaterialId {
            handle: r.handle,
            index: r.count,
        });
        if let Some(id) = callback {
            if result.is_ok() {
                self.callbacks.push(id);
            } else if let Ok(mut callbacks) = CALLBACKS.lock() {
                callbacks.remove(&id);
            }
        }
        result
    }
    pub fn clear_boxes(&mut self, material: MaterialId) -> Result<()> {
        let mut q = Request::new(op::CLEAR_BOXES);
        q.object = material.handle;
        self.call(q).map(|_| ())
    }
    pub fn add_box(&mut self, material: MaterialId, bounds: [f64; 6]) -> Result<()> {
        let mut q = Request::new(op::ADD_BOX);
        q.object = material.handle;
        q.values[..6].copy_from_slice(&bounds);
        self.call(q).map(|_| ())
    }
    pub fn create_box(
        &mut self,
        scene: Scene,
        id: i32,
        mass: f64,
        half: [f64; 3],
        pose: [f64; 7],
    ) -> Result<Body> {
        let mut q = Request::new(op::BOX);
        q.scene = scene.0;
        q.id = id;
        q.values[0] = mass;
        q.values[1..4].copy_from_slice(&half);
        q.values[4..11].copy_from_slice(&pose);
        self.call(q).map(|r| Body(r.handle))
    }
    pub fn create_sublevel(
        &mut self,
        scene: Scene,
        id: i32,
        pose: [f64; 7],
        bounds: [i32; 6],
        mass: Mass,
    ) -> Result<Body> {
        let mut q = Request::new(op::SUBLEVEL);
        q.scene = scene.0;
        q.id = id;
        q.bounds = bounds;
        q.values[0] = mass.mass;
        q.values[1..4].copy_from_slice(&mass.center);
        q.values[4..13].copy_from_slice(&mass.inertia);
        q.values[13..20].copy_from_slice(&pose);
        self.call(q).map(|r| Body(r.handle))
    }
    pub fn pose(&mut self, body: Body) -> Result<[f64; 7]> {
        let mut q = Request::new(op::POSE);
        q.object = body.0;
        self.call(q).map(|r| r.values[..7].try_into().unwrap())
    }
    pub fn velocity(&mut self, body: Body) -> Result<[f64; 6]> {
        let mut q = Request::new(op::VELOCITY);
        q.object = body.0;
        self.call(q).map(|r| r.values[..6].try_into().unwrap())
    }
    fn vector(
        &mut self,
        operation: u32,
        body: Body,
        linear: [f64; 3],
        angular: [f64; 3],
        wake: bool,
    ) -> Result<()> {
        let mut q = Request::new(operation);
        q.object = body.0;
        q.flags = u32::from(wake);
        q.values[..3].copy_from_slice(&linear);
        q.values[3..6].copy_from_slice(&angular);
        self.call(q).map(|_| ())
    }
    pub fn impulse(
        &mut self,
        body: Body,
        linear: [f64; 3],
        angular: [f64; 3],
        wake: bool,
    ) -> Result<()> {
        self.vector(op::IMPULSE, body, linear, angular, wake)
    }
    pub fn add_velocity(
        &mut self,
        body: Body,
        linear: [f64; 3],
        angular: [f64; 3],
        wake: bool,
    ) -> Result<()> {
        self.vector(op::ADD_VELOCITY, body, linear, angular, wake)
    }
    pub fn wake(&mut self, body: Body) -> Result<()> {
        let mut q = Request::new(op::WAKE);
        q.object = body.0;
        self.call(q).map(|_| ())
    }
    pub fn tick(&mut self, scene: Scene) -> Result<()> {
        let mut q = Request::new(op::TICK);
        q.scene = scene.0;
        self.call(q).map(|_| ())
    }
    pub fn step(&mut self, scene: Scene, dt: f64) -> Result<()> {
        let mut q = Request::new(op::STEP);
        q.scene = scene.0;
        q.values[0] = dt;
        self.call(q).map(|_| ())
    }
    pub fn upload_chunk(
        &mut self,
        scene: Scene,
        coordinate: [i32; 3],
        global: bool,
        body: Option<Body>,
        cells: &[abi::Cell],
    ) -> Result<()> {
        if cells.len() > MAX_CELLS {
            return Err(Error::InvalidInput);
        }
        let mut q = Request::new(op::UPLOAD);
        q.scene = scene.0;
        q.coord = coordinate;
        q.flags = u32::from(global);
        q.object = body.map_or(0, |b| b.0);
        q.data = cells.as_ptr().cast();
        q.length = cells.len() as u32;
        self.call(q).map(|_| ())
    }
    pub fn remove_chunk(&mut self, scene: Scene, coordinate: [i32; 3], global: bool) -> Result<()> {
        let mut q = Request::new(op::REMOVE_CHUNK);
        q.scene = scene.0;
        q.coord = coordinate;
        q.flags = u32::from(global);
        self.call(q).map(|_| ())
    }
    pub fn remove_body(&mut self, body: Body) -> Result<()> {
        let mut q = Request::new(op::REMOVE_BODY);
        q.object = body.0;
        self.call(q).map(|_| ())
    }
    pub fn fixed_joint(
        &mut self,
        scene: Scene,
        a: Option<Body>,
        b: Option<Body>,
        anchors: [f64; 6],
        rotation: [f64; 4],
    ) -> Result<Joint> {
        let mut q = Request::new(op::FIXED_JOINT);
        q.scene = scene.0;
        q.object = a.map_or(0, |b| b.0);
        q.other = b.map_or(0, |b| b.0);
        q.values[..6].copy_from_slice(&anchors);
        q.values[6..10].copy_from_slice(&rotation);
        self.call(q).map(|r| Joint(r.handle))
    }
    pub fn joint_sample(&mut self, joint: Joint) -> Result<(bool, [f64; 6])> {
        let mut q = Request::new(op::JOINT_SAMPLE);
        q.object = joint.0;
        self.call(q)
            .map(|r| (r.count != 0, r.values[..6].try_into().unwrap()))
    }
    pub fn remove_joint(&mut self, joint: Joint) -> Result<()> {
        let mut q = Request::new(op::REMOVE_JOINT);
        q.object = joint.0;
        self.call(q).map(|_| ())
    }
    pub fn collisions(&mut self, scene: Scene) -> Result<Vec<f64>> {
        let mut q = Request::new(op::CAPTURE_COLLISIONS);
        q.scene = scene.0;
        let size = self.call(q)?.count as usize;
        if size > u32::MAX as usize {
            return Err(Error::InvalidInput);
        }
        let mut values = vec![0.; size];
        let mut q = Request::new(op::COPY_COLLISIONS);
        q.owner = self.owner;
        q.scene = scene.0;
        let mut response = Response {
            data: values.as_mut_ptr(),
            capacity: size as u32,
            ..Default::default()
        };
        checked(unsafe { (self.api.execute)(&q, &mut response) })?;
        if !response.header.valid::<Response>()
            || response.written as usize != size
            || response.count as usize != size
        {
            return Err(Error::InvalidInput);
        }
        Ok(values)
    }
}
impl Drop for Bridge {
    fn drop(&mut self) {
        let _ = self.close();
    }
}
