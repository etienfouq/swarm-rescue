# -*- coding: utf-8 -*-

from statemachine import StateMachine, State


class DroneStateMachine(StateMachine):

    # States
    # idle = State('IDLE', initial=True)
    exploration = State('EXPLORATION', initial=True)
    rescue_victim = State('RESCUE_VICTIM')
    return_base = State('RETURN_BASE')
    drop_victim = State('DROP_VICTIM')
    repositionning = State('REPOSITIONNING')


    # Transitions
    # start_sm = idle.to(exploration)
    # stop_sm = exploration.to(idle)

    victim_found = exploration.to(rescue_victim)
    victim_grasped = rescue_victim.to(return_base)
    oriented_to_base = return_base.to(drop_victim)
    victim_dropped = drop_victim.to(repositionning)
    back_to_exploration = repositionning.to(exploration)


    #### NE PAS METTRE DE INIT A LA SM SAUF SI ON UTILISE L'OBJET MYDRONE


    # Callback functions
    def on_victim_found(self):
        print("A victim has been found")

    def on_victim_grasped(self):
        print("The victim has been grasped")

    def on_oriented_to_base(self):
        print("The victim has brought back to the base")

    def on_victim_dropped(self):
        print("The victim has been dropped")

    def on_back_to_exploration(self):
        print("The drone is ready to explore again")

