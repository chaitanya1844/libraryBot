# Published continuously by libbot_sorting's task manager so the
# control panel / RViz can show what the robot is doing.

uint8 IDLE = 0
uint8 AWAITING_RFID = 1
uint8 NAVIGATING_TO_SHELF = 2
uint8 LOCATING_SLOT = 3
uint8 PLACING_BOOK = 4
uint8 RETURNING_TO_STATION = 5
uint8 ERROR = 6

std_msgs/Header header
uint8 state
string tag_id
string target_shelf_id
string message
float32 progress          # 0-1 rough progress indicator for the current state
int32 books_sorted_total
