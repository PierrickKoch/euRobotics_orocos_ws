require "rfsm"
require "rttlib"
require "rfsm_rtt"

Supervisor = rtt.getTC()
local fsm

function configureHook()
  -- load state machine
  fsm = rfsm.init(dofile("/home/u0063509/src/svn/stacks/euRobotics_orocos_ws/youbot_supervisor/statemachine.lua"))

  -- the following creates a string input port, adds it as a event
  -- driven port to the Taskcontext.
  events_in = rtt.InputPort.new("string")
  Supervisor:addEventPort(events_in, "events", "rFSM event input port")
  -- connect the event port to e.g. the Controller component event
  -- output port
  -- events_in:connect(Supervisor:getPeer("Controller"):getPort("controller_event"))

  -- Generates a getevents function which returns all data on the current port as
  -- events. This function is called by the rFSM core to check for
  -- new events.
  fsm.getevents = rfsm_rtt.gen_read_events(events_in)

  -- optional: create a string port to which the currently active
  -- state of the FSM will be written. gen_write_fqn generates a
  -- function suitable to be added to the rFSM step hook to do this.
  fqn_out = rtt.OutputPort.new("string")
  Supervisor:addPort(fqn_out, "rFSM_cur_fqn", "current active rFSM state")
  fsm.step_hook=rfsm_rtt.gen_write_fqn(fqn_out)
  return true
end

function startHook()
   return true
end

function updateHook()
   rfsm.run(fsm)
   return true
end

function stopHook()
   return true
end

function cleanupHook()
   -- cleanup the created ports.
   Supervisor:removePort(fqn_out:info().name)
   Supervisor:removePort(events_in:info().name)
   fqn_out:delete()
   events_in:delete()
   return true
end

