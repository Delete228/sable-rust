//! Personal experimental adapter. The host's runtime worlds own every real
//! inventory/ticker; this mod retains only identities and derived physics data.
mod frames;
mod lifecycle;
pub mod lifecycle_backend;
pub mod owner_state;
mod persist;
mod scene;
use mod_api::{self, *};
use scene::{NativeImpulse, Scene};
use std::{cell::RefCell, sync::OnceLock};
#[derive(Default)]
struct Mod;
static BOOTSTRAP: OnceLock<TicketController> = OnceLock::new();
thread_local! {
    static OPEN_SCENE:RefCell<Option<Scene>>=const { RefCell::new(None) };
    static OPEN_RESET_PLAYER: RefCell<Option<PlayerRef>> = const { RefCell::new(None) };
    static STATE:RefCell<Option<State>>=const { RefCell::new(None) };
    static DISPOSED:RefCell<Option<Vec<String>>>=const {RefCell::new(None)};
}
struct State {
    zero: Scene,
    dynamic: Scene,
    lifecycle: Option<lifecycle::Controller>,
    native_request: Option<(u8, NativeImpulse)>,
    native_receipt: Option<String>,
    native_error: bool,
    repair_requested: bool,
    repair_locked: bool,
    repair_attempts: u32,
    repair_pending: Option<scene::SplitParentRepair>,
    repair_status: String,
}
impl State {
    fn start(host: &mut dyn ServerHost) -> Result<Self, String> {
        let bootstrap = *BOOTSTRAP
            .get()
            .ok_or_else(|| "source bootstrap controller unavailable".to_string())?;
        if bootstrap.id() != "sable_port:source_bootstrap" {
            return Err("source bootstrap controller registration was refused".into());
        }
        let zero = Scene::start(host, bootstrap)?;
        let dynamic = Scene::dynamic(host, &zero)?;
        Ok(Self {
            zero,
            dynamic,
            lifecycle: Some(lifecycle::Controller::default()),
            native_request: None,
            native_receipt: None,
            native_error: false,
            repair_requested: false,
            repair_locked: false,
            repair_attempts: 0,
            repair_pending: None,
            repair_status: r#"{"pending":false,"staged":false,"committed":false,"barrier_epoch":null,"restart_required":false,"problem":null}"#.into(),
        })
    }
    fn status(&self, host: &dyn ServerHost) -> String {
        let mut zero = self.zero.status(host);
        zero.pop();
        let lifecycle = self
            .lifecycle
            .as_ref()
            .map(|c| c.status())
            .unwrap_or_else(|| r#"{"pending":true,"phase":"executing_frontier"}"#.into());
        format!(
            "{zero},\"dynamic\":{},\"lifecycle\":{lifecycle},\"native_control\":{{\"pending\":{},\"error\":{},\"last\":{}}},\"split_repair\":{}}}",
            self.dynamic.status(host),
            self.native_request.is_some(),
            self.native_error,
            self.native_receipt.as_deref().unwrap_or("null"),
            self.repair_status,
        )
    }
}
/// Revalidate the adapter identity/frame/pause inputs after actual host
/// callbacks, without retaining a STATE borrow across any host operation.
fn validate_policy_slots(expected: &[scene::Slot]) -> Result<(), String> {
    STATE.with(|slot| {
        let slot = slot
            .try_borrow()
            .map_err(|_| "policy observation reentry".to_string())?;
        let state = slot
            .as_ref()
            .ok_or_else(|| "policy owner session disappeared".to_string())?;
        for expected in expected {
            let matches = state
                .zero
                .matches_policy_slot(expected)
                .or_else(|| state.dynamic.matches_policy_slot(expected));
            if matches != Some(true) {
                return Err(
                    "actual adapter owner/frame/pause state changed during host callbacks".into(),
                );
            }
        }
        Ok(())
    })
}
/// Keep interactive replies short; the full diagnostic remains in the log.
fn open_demo_error_line(problem: &str) -> String {
    let message = match problem {
        "reset observer overlaps current or initial projection" => {
            "Move away from the structure to reset."
        }
        "open demo start requires healthy paused native epoch zero; reset first" => {
            "Reset before starting again."
        }
        "open demo waits for its actual restored Source and live binding" => {
            "Scene still loading; try again."
        }
        "existing review-v3 Open Source requires explicit reset to OpenDemo-v4 before start" => {
            "Reset the old demo before starting."
        }
        "reentrant" => "Scene is busy; try again.",
        "open demo inactive" => "Open the demo world first.",
        other => other,
    };
    let compact = message.split_whitespace().collect::<Vec<_>>().join(" ");
    let mut line = String::from("Sable: ");
    line.extend(compact.chars().take(64));
    if compact.chars().count() > 64 {
        line.push('…');
    }
    line
}
#[mod_api::sunrise_mod]
impl Hooks for Mod {
    fn register(&self, registrar: &mut dyn Registrar, _: &RegistrationContext) {
        let _ = registrar.register_attachment_type("sable_port:motion", false);
        let _ = registrar.register_attachment_type("sable_port:owner", false);
        let _ = registrar.register_attachment_type("sable_port:lifecycle", false);
        let _ = BOOTSTRAP.set(registrar.register_ticket_controller("source_bootstrap", false));
    }
    fn register_commands(&self, _: &mut dyn ServerHost, event: &mut RegisterCommandsEvent<'_>) {
        for action in ["status", "start", "reset", "checkpoint"] {
            event
                .commands
                .push(Command::op(&format!("sabledemo {action}"), 2));
        }
        for action in ["status", "pause", "resume", "step", "checkpoint", "reset"] {
            event
                .commands
                .push(Command::op(&format!("sablephase3 {action}"), 2));
        }
        for pattern in [
            "sablephase3 reconcile-split-parent",
            "sablephase3 lifecycle capture <slot:word> <seed:block_pos>",
            "sablephase3 lifecycle rearm <slot:word>",
            "sablephase3 lifecycle return <slot:word>",
            "sablephase3 lifecycle split <slot:word> <child:word>",
            "sablephase3 lifecycle retire <slot:word>",
            "sablephase3 lifecycle dispose",
            "sablephase3 lifecycle status",
            "sablephase3 native spin <slot:word>",
            "sablephase3 native drift <slot:word>",
            "sablephase3 native fast-drift <slot:word>",
        ] {
            event.commands.push(Command::op(pattern, 2));
        }
        for action in [
            "status",
            "pause",
            "release",
            "step",
            "impulse",
            "checkpoint",
            "reset",
        ] {
            event
                .commands
                .push(Command::op(&format!("sablephase3 dynamic {action}"), 2));
        }
    }
    fn live_world_admission(
        &self,
        host: &mut dyn ServerHost,
        event: &mut LiveWorldAdmissionEvent<'_>,
    ) -> Flow {
        let open = OPEN_SCENE.with(|slot| {
            let Ok(slot) = slot.try_borrow() else {
                return Some(Flow::Cancel);
            };
            slot.as_ref()
                .and_then(|scene| scene.admit(host, event.live_world, event.changes))
                .map(|result| match result {
                    Ok(()) => Flow::Continue,
                    Err(problem) => {
                        eprintln!("[sable_open_demo] edit refused: {problem}");
                        Flow::Cancel
                    }
                })
        });
        if let Some(flow) = open {
            return flow;
        }
        STATE.with(|slot| {
            let Ok(slot) = slot.try_borrow() else {
                eprintln!("[sable_phase3] reentrant admission refused");
                return Flow::Cancel;
            };
            let Some(state) = slot.as_ref() else {
                return Flow::Continue;
            };
            for scene in [&state.zero, &state.dynamic] {
                if let Some(result) = scene.admit(host, event.live_world, event.changes) {
                    return match result {
                        Ok(()) => Flow::Continue,
                        Err(problem) => {
                            eprintln!(
                                "[sable_phase3] prospective edit refused before mutation: {problem}"
                            );
                            Flow::Cancel
                        }
                    };
                }
            }
            Flow::Continue
        })
    }
    fn run_mod_command(
        &self,
        host: &mut dyn ServerHost,
        event: &mut RunModCommandEvent<'_>,
    ) -> Flow {
        if event.command.literals.first().map(String::as_str) == Some("sabledemo") {
            let action = event
                .command
                .literals
                .get(1)
                .map(String::as_str)
                .unwrap_or("");
            let outcome = OPEN_SCENE.with(|slot| {
                let mut slot = slot.try_borrow_mut().map_err(|_| "reentrant".to_string())?;
                let scene = slot
                    .as_mut()
                    .ok_or_else(|| "open demo inactive".to_string())?;
                scene.open_demo_control(host, action)?;
                if action == "reset" {
                    OPEN_RESET_PLAYER.with(|pending| {
                        *pending.borrow_mut() = match event.source {
                            CommandSource::Player { player, .. } => Some(*player),
                            _ => None,
                        };
                    });
                }
                Ok::<_, String>(match action {
                    "status" => scene.open_demo_status(host),
                    "start" => "Sable: started.".into(),
                    "reset" => "Sable: reset requested.".into(),
                    _ => "Sable: checkpoint requested.".into(),
                })
            });
            let reply = match outcome {
                Ok(reply) => reply,
                Err(problem) => {
                    eprintln!("[sable_open_demo] {action} refused: {problem}");
                    if action == "status" {
                        format!(r#"{{"schema":"sable.open-demo.status","error":{problem:?}}}"#)
                    } else {
                        open_demo_error_line(&problem)
                    }
                }
            };
            // Player system messages remain visible with command feedback off.
            // Explicit status keeps the existing machine-readable feedback path.
            if action != "status"
                && let CommandSource::Player { player, .. } = event.source
            {
                match player.tell(host, Text::literal(reply.clone())) {
                    Ok(true) => return Flow::Cancel,
                    Ok(false) => eprintln!("[sable_open_demo] command player unavailable"),
                    Err(problem) => {
                        eprintln!("[sable_open_demo] player reply refused: {problem:?}")
                    }
                }
            }
            if let Err(problem) = host.commands().feedback(Text::literal(reply)) {
                eprintln!("[sable_open_demo] feedback refused: {problem:?}");
            }
            return Flow::Cancel;
        }
        if event.command.literals.first().map(String::as_str) != Some("sablephase3") {
            return Flow::Continue;
        }
        STATE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                eprintln!("[sable_phase3] reentrant control refused");
                let _ = host.commands().feedback(Text::literal(
                    "{\"schema\":\"sable.phase3.status\",\"error\":true,\"reentrant\":true}",
                ));
                return;
            };
            if slot.is_none() {
                let reply=DISPOSED.with(|disposed|disposed.borrow().as_ref().map(|sources|format!(r#"{{"schema":"sable.phase3.status","native_owners_dropped":true,"levels_closed":{},"disposed":true}}"#,sources.iter().all(|id|host.level(id).is_none()))));
                if let Some(reply)=reply {drop(slot);let _=host.commands().feedback(Text::literal(reply));return;}
            }
            let result = slot
                .as_mut()
                .ok_or_else(|| "review session inactive".to_string())
                .and_then(|state| {
                    if event.command.literals.get(1).map(String::as_str)==Some("reconcile-split-parent") {
                        if state.repair_locked || state.native_request.is_some()
                            || state.lifecycle.as_ref().is_none_or(|controller|controller.busy())
                            || !state.zero.failed || state.dynamic.failed
                            || state.zero.lifecycle_slots().iter().any(|slot|slot.native_bodies!=0||slot.world.is_some())
                            || state.dynamic.lifecycle_slots().iter().any(|slot|!slot.paused||slot.epoch!=0)
                        {return Err("explicit split repair requires the fresh failed/no-controlled-body session and no pending operation".into());}
                        state.repair_locked=true;state.repair_requested=true;state.repair_attempts=0;
                        state.repair_status=r#"{"pending":true,"staged":false,"committed":false,"barrier_epoch":null,"restart_required":true,"problem":null}"#.into();
                        return Ok(());
                    }
                    if state.repair_locked && event.command.literals.last().map(String::as_str)!=Some("status") {
                        return Err("split repair session permits status then full quit/reopen only".into());
                    }
                    if event.command.literals.get(1).map(String::as_str)==Some("native") {
                        if state.lifecycle.as_ref().is_none_or(|c|c.busy()) {return Err("native control refuses an unresolved lifecycle frontier".into());}
                        let Some(CommandArgumentValue::String(value))=event.command.argument("slot") else{return Err("missing native Source slot".into())};
                        let number=match value.as_str(){"translate"=>0,"yaw"=>1,"pitch"=>2,"dynamic"=>3,_=>return Err("unknown native Source slot".into())};
                        if number==3 || state.native_request.is_some() {return Err("native review impulse requires a free zeroG Source turn".into());}
                        let action=event.command.literals.get(2).map(String::as_str).unwrap_or("");
                        let input=NativeImpulse::parse(action).ok_or_else(||"unknown native impulse".to_string())?;
                        state.native_request=Some((number,input));state.native_receipt=None;state.native_error=false;
                        return Ok(());
                    }
                    if event.command.literals.get(1).map(String::as_str)==Some("lifecycle") {
                        let action=event.command.literals.get(2).map(String::as_str).unwrap_or("");
                        if action=="status" {return Ok(());}
                        if state.native_request.is_some(){return Err("lifecycle waits for the queued actual native impulse".into());}
                        let slot=|name:&str|->Result<u8,String> {
                            let Some(CommandArgumentValue::String(value))=event.command.argument(name) else{return Err("missing resolved Source slot".into())};
                            match value.as_str(){"translate"=>Ok(0),"yaw"=>Ok(1),"pitch"=>Ok(2),"dynamic"=>Ok(3),_=>Err("slot must be translate/yaw/pitch/dynamic".into())}
                        };
                        let request=match action {
                            "capture"=>{let Some(CommandArgumentValue::BlockPos(seed))=event.command.argument("seed") else{return Err("missing actual resolved capture seed".into())};lifecycle::Request::Capture{slot:slot("slot")?,seed:*seed}},
                            "rearm"=>lifecycle::Request::Rearm{slot:slot("slot")?},
                            "return"=>lifecycle::Request::Return{slot:slot("slot")?},
                            "split"=>lifecycle::Request::Split{slot:slot("slot")?,child:slot("child")?},
                            "retire"=>lifecycle::Request::Retire{slot:slot("slot")?},
                            "dispose"=>lifecycle::Request::Dispose,
                            _=>return Err("unknown lifecycle command".into()),
                        };
                        return state.lifecycle.as_mut().ok_or_else(||"lifecycle frontier is executing".to_string())?.queue(request);
                    }
                    let dynamic =
                        event.command.literals.get(1).map(String::as_str) == Some("dynamic");
                    let action = event
                        .command
                        .literals
                        .get(if dynamic { 2 } else { 1 })
                        .map(String::as_str)
                        .unwrap_or("");
                    if action!="status" && (state.native_request.is_some() || state.lifecycle.as_ref().is_none_or(|c|c.busy())) {return Err("physical control waits for the current lifecycle frontier".into());}
                    if dynamic {
                        state.dynamic.control(host, action)
                    } else {
                        state.zero.control(host, action)
                    }
                });
            let reply = match result {
                Ok(()) => slot.as_ref().unwrap().status(host),
                Err(problem) => {
                    eprintln!("[sable_phase3] control refused: {problem}");
                    "{\"schema\":\"sable.phase3.status\",\"error\":true}".into()
                }
            };
            eprintln!("[sable_phase3] {reply}");
            drop(slot);
            if let Err(problem) = host.commands().feedback(Text::literal(reply)) {
                eprintln!("[sable_phase3] status feedback refused: {problem:?}");
            }
        });
        Flow::Cancel
    }
    fn server_tick(&self, host: &mut dyn ServerHost, event: &mut ServerTickEvent<'_>) {
        let mut reset_reply = None;
        OPEN_SCENE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                return;
            };
            if let Some(scene) = slot.as_mut() {
                if let ServerTickPhase::ServerTickStart { world_running } = event.phase {
                    scene.world_running = world_running;
                    let resetting = scene.reset_pending();
                    let result = scene
                        .settle_auto_retirements(host)
                        .and_then(|()| scene.activate_sources(host))
                        .and_then(|()| {
                            if scene.reset_pending() {
                                scene.reset_episode(host)
                            } else {
                                Ok(())
                            }
                        });
                    if resetting && let Err(problem) = &result {
                        scene.cancel_open_demo_reset(problem);
                    }
                    if resetting && !scene.reset_pending() {
                        reset_reply = OPEN_RESET_PLAYER.with(|pending| {
                            pending.borrow_mut().take().map(|player| {
                                let reply = match &result {
                                    Ok(()) => "Sable: reset complete.".into(),
                                    Err(problem) => open_demo_error_line(problem),
                                };
                                (player, reply)
                            })
                        });
                    }
                    if let Err(problem) = result {
                        scene.failed = true;
                        eprintln!("[sable_open_demo] preparation refused: {problem}");
                    }
                }
                if matches!(event.phase, ServerTickPhase::ServerTickEnd { .. })
                    && scene.checkpoint_requested
                {
                    scene.checkpoint_requested = false;
                    if let Err(problem) = scene.checkpoint(host) {
                        scene.failed = true;
                        eprintln!("[sable_open_demo] checkpoint refused: {problem}");
                    }
                }
            }
        });
        if let Some((player, reply)) = reset_reply {
            match player.tell(host, Text::literal(reply)) {
                Ok(true) => {}
                Ok(false) => eprintln!("[sable_open_demo] reset reply player unavailable"),
                Err(problem) => eprintln!("[sable_open_demo] reset reply refused: {problem:?}"),
            }
        }
        if matches!(event.phase, ServerTickPhase::ServerTickStart { .. }) {
            let work = STATE.with(|slot| {
                let mut slot = slot.try_borrow_mut().ok()?;
                let state = slot.as_mut()?;
                if !state.repair_requested {
                    return None;
                }
                state.repair_attempts = state.repair_attempts.saturating_add(1);
                Some((state.repair_pending.take(), state.repair_attempts))
            });
            if let Some((mut pending, attempts)) = work {
                // No STATE borrow crosses restore, transaction or common save.
                let result = (|| -> Result<Option<u64>, String> {
                    if attempts > 200 {
                        return Err("explicit split repair readiness/ACK budget exhausted".into());
                    }
                    if pending.is_none() {
                        pending = scene::SplitParentRepair::prepare(host)?;
                    }
                    match pending.as_mut() {
                        Some(repair) => repair.advance(host),
                        None => Ok(None),
                    }
                })();
                let staged = pending.as_ref().is_some_and(|repair| repair.staged);
                let (waiting, committed, barrier, problem) = match result {
                    Ok(Some(barrier)) => (false, true, barrier.to_string(), "null".to_string()),
                    Ok(None) => (true, false, "null".into(), "null".into()),
                    Err(problem) => {
                        // All repair errors are fixed ASCII policy/host diagnostics.
                        eprintln!("[sable_phase3] explicit split parent repair refused: {problem}");
                        (false, false, "null".into(), format!("{problem:?}"))
                    }
                };
                let status = format!(
                    r#"{{"pending":{waiting},"staged":{staged},"committed":{committed},"barrier_epoch":{barrier},"restart_required":true,"attempts":{attempts},"problem":{problem}}}"#
                );
                eprintln!("[sable_phase3] explicit_split_parent_repair {status}");
                STATE.with(|slot| {
                    if let Ok(mut slot) = slot.try_borrow_mut()
                        && let Some(state) = slot.as_mut()
                    {
                        state.repair_requested = waiting;
                        state.repair_pending = pending;
                        state.repair_status = status;
                    }
                });
            }
        }
        STATE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                eprintln!("[sable_phase3] reentrant hook refused");
                return;
            };
            if let Some(state) = slot.as_mut() {
                if matches!(event.phase, ServerTickPhase::ServerTickStart { .. })
                    && let Some((number, input)) = state.native_request.take()
                {
                    match state.zero.native_impulse(host, number, input) {
                        Ok(receipt) => {
                            state.native_receipt = Some(receipt);
                            state.native_error = false;
                        }
                        Err(problem) => {
                            state.native_error = true;
                            eprintln!("[sable_phase3] queued native impulse refused: {problem}");
                        }
                    }
                }
                for scene in [&mut state.zero, &mut state.dynamic] {
                    if let ServerTickPhase::ServerTickStart { world_running } = event.phase {
                        scene.world_running = world_running;
                        if let Err(problem) = scene.settle_auto_retirements(host) {
                            scene.failed = true;
                            eprintln!(
                                "[sable_phase3] actual empty-retirement witness refused: {problem}"
                            );
                        }
                        if let Err(problem) = scene.activate_sources(host) {
                            scene.failed = true;
                            eprintln!("[sable_phase3] source activation refused: {problem}");
                        }
                        if scene.reset_pending()
                            && let Err(problem) = scene.reset_episode(host)
                        {
                            eprintln!("[sable_phase3] explicit reset refused: {problem}");
                        }
                    }
                    if matches!(event.phase, ServerTickPhase::ServerTickEnd { .. })
                        && scene.checkpoint_requested
                    {
                        scene.checkpoint_requested = false;
                        if let Err(problem) = scene.checkpoint(host) {
                            eprintln!("[sable_phase3] checkpoint refused: {problem}");
                        }
                    }
                }
            }
        });
        if matches!(event.phase, ServerTickPhase::ServerTickStart { .. }) {
            // Extract only the policy controller. STATE retains the actual
            // scenes so ordinary edit-admission callbacks can borrow them.
            let work = STATE.with(|slot| {
                let mut slot = slot.try_borrow_mut().ok()?;
                let state = slot.as_mut()?;
                let controller = state.lifecycle.take()?;
                let mut sources = state.zero.lifecycle_slots();
                sources.extend(state.dynamic.lifecycle_slots());
                Some((controller, sources))
            });
            if let Some((mut controller, sources)) = work {
                if let Some(bootstrap) = BOOTSTRAP.get().copied() {
                    controller.turn(host, &sources, bootstrap);
                }
                let dispose = controller.is_disposed();
                let updates = std::mem::take(&mut controller.updates);
                let dropped=STATE.with(|slot| {
                    let Ok(mut slot)=slot.try_borrow_mut() else {eprintln!("[sable_phase3] lifecycle cache application reentry; restart required");return None};
                    let state=slot.as_mut()?;
                    for scene in [&mut state.zero,&mut state.dynamic] {
                        if let Err(problem)=scene.install_records(host,&updates) {scene.failed=true;eprintln!("[sable_phase3] durable ownership awaits cache reconciliation: {problem}");}
                    }
                    state.lifecycle=Some(controller);
                    if dispose {slot.take()}else{None}
                });
                if let Some(state) = dropped {
                    let ids = sources.into_iter().map(|s| s.source_id).collect();
                    drop(state); // Releases both actual native scenes and their shared owner.
                    DISPOSED.with(|slot| *slot.borrow_mut() = Some(ids));
                }
            }
        }
    }
    fn server_started(&self, host: &mut dyn ServerHost, _: &mut ServerStartedEvent) {
        if std::env::var_os("SABLE_PHASE3_OPEN_DEMO").as_deref() == Some(std::ffi::OsStr::new("1"))
        {
            let result = BOOTSTRAP
                .get()
                .copied()
                .ok_or_else(|| "bootstrap unavailable".to_string())
                .and_then(|bootstrap| Scene::open_demo(host, bootstrap));
            match result {
                Ok(scene) => OPEN_SCENE.with(|slot| *slot.borrow_mut() = Some(scene)),
                Err(problem) => eprintln!("[sable_open_demo] start refused: {problem}"),
            };
            return;
        }
        if std::env::var_os("SABLE_PHASE3_REVIEW").as_deref() != Some(std::ffi::OsStr::new("1")) {
            return;
        }
        match State::start(host) {
            Ok(state) => STATE.with(|slot| {
                DISPOSED.with(|slot| slot.borrow_mut().take());
                if let Ok(mut slot) = slot.try_borrow_mut() {
                    *slot = Some(state);
                } else {
                    eprintln!("[sable_phase3] reentrant start refused");
                }
            }),
            Err(problem) => eprintln!("[sable_phase3] start refused: {problem}"),
        }
    }
    fn level_tick_pre(&self, host: HostSide, event: &mut LevelTickPreEvent) {
        let HostSide::Server(host) = host else {
            return;
        };
        OPEN_SCENE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                return;
            };
            if let Some(scene) = slot.as_mut()
                && let Err(problem) = scene.tick(host, event.level)
            {
                scene.failed = true;
                eprintln!("[sable_open_demo] publication paused: {problem}");
            }
        });
        STATE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                eprintln!("[sable_phase3] reentrant hook refused");
                return;
            };
            if let Some(state) = slot.as_mut() {
                if state.lifecycle.as_ref().is_some_and(|c| c.unresolved()) {
                    return;
                }
                for (index, scene) in [&mut state.zero, &mut state.dynamic]
                    .into_iter()
                    .enumerate()
                {
                    if let Err(problem) = scene.tick(host, event.level) {
                        scene.failed = true;
                        eprintln!("[sable_phase3] scene={index} publication paused: {problem}");
                    }
                }
            }
        });
    }
    fn server_stopping(&self, host: &mut dyn ServerHost, _: &mut ServerStoppingEvent) {
        OPEN_SCENE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                return;
            };
            if let Some(scene) = slot.as_mut() {
                match scene
                    .checkpoint(host)
                    .and_then(|()| scene.final_save_witness(host))
                {
                    Ok(witness) => eprintln!("[sable_open_demo] {witness}"),
                    Err(problem) => eprintln!("[sable_open_demo] final save refused: {problem}"),
                }
                if let Err(problem) = scene.release_bootstrap_on_stop(host) {
                    eprintln!("[sable_open_demo] bootstrap release refused: {problem}");
                }
            }
        });
        STATE.with(|slot| {
            let Ok(mut slot) = slot.try_borrow_mut() else {
                eprintln!("[sable_phase3] reentrant hook refused");
                return;
            };
            if let Some(state) = slot.as_mut() {
                for scene in [&mut state.zero, &mut state.dynamic] {
                    match scene.checkpoint(host) {
                        Ok(())=>match scene.final_save_witness(host) {
                            Ok(witness)=>eprintln!("[sable_phase3] {witness}"),
                            Err(problem)=>eprintln!("[sable_phase3] completed checkpoint has no final-save readback witness: {problem}"),
                        },
                        Err(problem)=>eprintln!("[sable_phase3] checkpoint refused: {problem}"),
                    }
                    if let Err(problem) = scene.release_bootstrap_on_stop(host) {
                        eprintln!("[sable_phase3] owned bootstrap cleanup refused: {problem}");
                    }
                }
            }
        });
    }
    fn server_stopped(&self, _: &mut dyn ServerHost, _: &mut ServerStoppedEvent) {
        OPEN_RESET_PLAYER.with(|pending| {
            pending.borrow_mut().take();
        });
        OPEN_SCENE.with(|slot| {
            slot.borrow_mut().take();
        });
        STATE.with(|slot| {
            if let Ok(mut slot) = slot.try_borrow_mut() {
                slot.take();
            } else {
                eprintln!("[sable_phase3] reentrant stop refused; process restart required");
            }
        });
    }
}

#[cfg(test)]
mod open_demo_command_tests {
    use super::open_demo_error_line;

    #[test]
    fn actionable_failures_fit_a_chat_line() {
        assert_eq!(
            open_demo_error_line("reset observer overlaps current or initial projection"),
            "Sable: Move away from the structure to reset."
        );
        assert_eq!(
            open_demo_error_line(
                "open demo start requires healthy paused native epoch zero; reset first"
            ),
            "Sable: Reset before starting again."
        );
    }

    #[test]
    fn unknown_diagnostics_are_single_line_bounded_and_unicode_safe() {
        let problem = format!("\n\tNative\r\n{}\tfailed\n", "é".repeat(100));
        let line = open_demo_error_line(&problem);
        assert_eq!(line.lines().count(), 1);
        assert!(line.chars().count() <= 72);
        assert!(line.starts_with("Sable: Native é"));
        assert!(line.ends_with('…'));
        assert_eq!(open_demo_error_line("Read\n\tfailed"), "Sable: Read failed");
    }
}
