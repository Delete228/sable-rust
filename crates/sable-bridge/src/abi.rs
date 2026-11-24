//! Versioned C-layout protocol. All pointers borrow caller-owned storage for one
//! call; no Rust allocation, type layout or allocator ownership crosses the ABI.
use std::ffi::c_void;

pub const ABI_VERSION: u32 = 1;
pub const MAX_CELLS: usize = 4096;
pub const MAX_BOXES: usize = 32768;

#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Header {
    pub version: u32,
    pub size: u32,
}
impl Header {
    pub const fn of<T>() -> Self {
        Self {
            version: ABI_VERSION,
            size: std::mem::size_of::<T>() as u32,
        }
    }
    pub fn valid<T>(&self) -> bool {
        *self == Self::of::<T>()
    }
}

pub mod status {
    pub const OK: u32 = 0;
    pub const INVALID: u32 = 1;
    pub const VERSION: u32 = 2;
    pub const BUSY: u32 = 3;
    pub const STALE: u32 = 4;
    pub const THREAD: u32 = 5;
    pub const REENTRANT: u32 = 6;
    pub const PANIC: u32 = 7;
    pub const BUFFER: u32 = 8;
    pub const CALLBACK: u32 = 9;
    pub const WRONG_SCENE: u32 = 10;
    pub const EXISTS: u32 = 11;
}

pub mod op {
    pub const OPEN: u32 = 1;
    pub const CLOSE: u32 = 2;
    pub const SCENE: u32 = 3;
    pub const DISPOSE_SCENE: u32 = 4;
    pub const CONFIGURE: u32 = 5;
    pub const MATERIAL: u32 = 6;
    pub const CLEAR_BOXES: u32 = 7;
    pub const ADD_BOX: u32 = 8;
    pub const BOX: u32 = 9;
    pub const SUBLEVEL: u32 = 10;
    pub const POSE: u32 = 11;
    pub const VELOCITY: u32 = 12;
    pub const IMPULSE: u32 = 13;
    pub const ADD_VELOCITY: u32 = 14;
    pub const WAKE: u32 = 15;
    pub const TICK: u32 = 16;
    pub const STEP: u32 = 17;
    pub const UPLOAD: u32 = 18;
    pub const REMOVE_CHUNK: u32 = 19;
    pub const REMOVE_BODY: u32 = 20;
    pub const FIXED_JOINT: u32 = 21;
    pub const JOINT_SAMPLE: u32 = 22;
    pub const REMOVE_JOINT: u32 = 23;
    pub const CAPTURE_COLLISIONS: u32 = 24;
    pub const COPY_COLLISIONS: u32 = 25;
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Cell {
    pub position: [u8; 3],
    pub state: u8,
    pub reserved: u32,
    pub material: u64,
}

pub type WorldFn = unsafe extern "C" fn(*const WorldRequest, *mut WorldResponse) -> u32;
pub type ContactFn = unsafe extern "C" fn(u64, *const Contact, WorldFn, *mut ContactReply) -> u32;
pub type ExecuteFn = unsafe extern "C" fn(*const Request, *mut Response) -> u32;

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Request {
    pub header: Header,
    pub operation: u32,
    pub flags: u32,
    pub owner: u64,
    pub scene: u64,
    pub object: u64,
    pub other: u64,
    pub id: i32,
    pub coord: [i32; 3],
    pub bounds: [i32; 6],
    pub values: [f64; 32],
    pub data: *const c_void,
    pub length: u32,
    pub reserved: u32,
    pub contact: Option<ContactFn>,
    pub context: u64,
}
impl Request {
    pub fn new(operation: u32) -> Self {
        Self {
            header: Header::of::<Self>(),
            operation,
            flags: 0,
            owner: 0,
            scene: 0,
            object: 0,
            other: 0,
            id: 0,
            coord: [0; 3],
            bounds: [0; 6],
            values: [0.; 32],
            data: std::ptr::null(),
            length: 0,
            reserved: 0,
            contact: None,
            context: 0,
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy)]
pub struct Response {
    pub header: Header,
    pub handle: u64,
    pub count: u64,
    pub values: [f64; 32],
    pub data: *mut f64,
    pub capacity: u32,
    pub written: u32,
}
impl Default for Response {
    fn default() -> Self {
        Self {
            header: Header::of::<Self>(),
            handle: 0,
            count: 0,
            values: [0.; 32],
            data: std::ptr::null_mut(),
            capacity: 0,
            written: 0,
        }
    }
}

#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct Contact {
    pub header: Header,
    pub block: [i32; 3],
    pub other: [i32; 3],
    pub point: [f64; 3],
    pub speed: f64,
    pub has_other: u32,
    pub reserved: u32,
    pub world: u64,
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ContactReply {
    pub header: Header,
    pub tangent: [f64; 3],
    pub reject: u32,
    pub reserved: u32,
}
impl Default for ContactReply {
    fn default() -> Self {
        Self {
            header: Header::of::<Self>(),
            tangent: [0.; 3],
            reject: 0,
            reserved: 0,
        }
    }
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WorldRequest {
    pub header: Header,
    pub world: u64,
    pub operation: u32,
    pub position: [i32; 3],
}
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct WorldResponse {
    pub header: Header,
    pub present: u32,
    pub material: u32,
    pub state: u32,
    pub reserved: u32,
}
impl Default for WorldResponse {
    fn default() -> Self {
        Self {
            header: Header::of::<Self>(),
            present: 0,
            material: 0,
            state: 0,
            reserved: 0,
        }
    }
}
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Api {
    pub header: Header,
    pub execute: ExecuteFn,
}
pub type GetApiFn = unsafe extern "C" fn(u32, u32, *mut Api) -> u32;
