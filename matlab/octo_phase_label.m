function label=octo_phase_label(phase)
switch phase
 case 'MOVE_1',label='Approaching above the object';
 case 'MOVE_2',label='Lowering the hand to the object';
 case 'WRAP',label='Wrapping fingers and engaging suction';
 case 'HOLD_TEST',label='Checking grip capacity';
 case 'MOVE_3',label='Lifting the object';
 case 'LIFT_SETTLE',label='Checking measured object lift';
 case 'MOVE_4',label='Carrying the object to the destination';
 case 'MOVE_5',label='Lowering the object at the destination';
 case 'SETDOWN',label='Confirming physical support contact';
 case 'VENT',label='Releasing suction and opening one digit at a time';
 case 'MOVE_6',label='Retreating after release';
 case 'SETTLE',label='Waiting for the object to settle';
 case 'DONE',label='Task complete';
 case 'FAILED',label='Task stopped';
 otherwise,label=phase;
end
end
