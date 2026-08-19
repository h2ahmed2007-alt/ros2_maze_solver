Toggle Wall Service — Completed

Service name:
 /toggle_walls_1_2

Service type:
 std_srvs/srv/SetBool

Request:
 bool data

Behavior:
 data = true
 → Wall 1 goes UP to 2.00 m
 → Wall 2 goes DOWN to 0.00 m

 data = false
 → Wall 1 goes DOWN to 0.00 m
 → Wall 2 goes UP to 2.00 m

Test:
Both commands were tested successfully from the terminal.

true:
success=True
"Commanded: wall 1 -> up (2.00 m), wall 2 -> down (0.00 m)."

false:
success=True
"Commanded: wall 1 -> down (0.00 m), wall 2 -> up (2.00 m)."
