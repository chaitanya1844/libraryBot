# A single RFID tag read, produced by the simulated RFID reader
# when a tagged book model in Gazebo comes within read range.

std_msgs/Header header

string tag_id            # unique RFID EPC/UID of the book, e.g. "BOOK-000042"
string book_title         # human readable title (sim metadata)
string genre               # sim metadata, used for shelf lookup fallback
string target_shelf_id    # resolved destination shelf id, e.g. "SHELF-A3"
float64 rssi               # simulated signal strength (higher = closer), dBm-like
float64 distance           # distance from reader to tag source, meters
