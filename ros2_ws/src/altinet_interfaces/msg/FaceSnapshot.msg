# Face snapshot captured from a tracked person.
std_msgs/Header header
string room_id
int32 track_id
float32 x
float32 y
float32 w
float32 h
float32 quality_score
float32 sharpness
float32 brightness
float32 pose_score
float32[10] landmarks
float32[] embedding
uint32 face_width
uint32 face_height
string face_encoding
uint8[] face_data
