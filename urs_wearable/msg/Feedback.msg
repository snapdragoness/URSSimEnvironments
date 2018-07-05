# Intermediate States
uint8 STATUS_PENDING=0
uint8 STATUS_ACTIVE=1
uint8 STATUS_REPLANNED=2

# Terminal States
uint8 STATUS_REJECTED=2
uint8 STATUS_PREEMPTED=3
uint8 STATUS_ABORTED=4
uint8 STATUS_SUCCEEDED=5

uint8 executor_id
uint8 status
Action current_action
string message
