return rfsm.composite_state:new {

  stop = rfsm.simple_state:new{
    entry = function()
      print("stop entry function completed")
    end,
    exit = function() 
      print("stop exit function completed") 
    end,
  },
  to_wall = rfsm.simple_state:new{ 
    -- set the goal pose equal to the current pose plus some offset in z direction 
    -- switch to the hover state upon reaching the goal position
    entry = function(fsm) 
      print("to_wall entry function completed") 
    end,
    doo = function()
      print("to_wall doo function started")
    end,
    exit = function() 
      print("to_wall exit function completed") 
    end,
  },
  follow_wall = rfsm.simple_state:new{ 
    -- set the goal pose equal to the current pose
    doo = function()
      print("follow_wall doo function started") 
    end,
    exit = function()
      print("follow_wall exit function completed") 
    end,
  },

  rfsm.trans:new{ src='initial', tgt='stop' },
  rfsm.trans:new{ src='stop', tgt='to_wall', events={'e_to_wall'} },
  rfsm.trans:new{ src='stop', tgt='follow_wall', events={'e_follow_wall'} },
  rfsm.trans:new{ src='to_wall', tgt='follow_wall', events={'e_follow_wall'} },
  rfsm.trans:new{ src='follow_wall', tgt='to_wall', events={'e_to_wall'} },
  rfsm.trans:new{ src='to_wall', tgt='stop', events={'e_stop'} },
  rfsm.trans:new{ src='follow_wall', tgt='stop', events={'e_stop'} },
}
