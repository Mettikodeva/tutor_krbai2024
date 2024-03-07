import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
from ultralytics.utils.checks import check_imshow
from ultralytics.utils.plotting import Annotator, colors
from copy import deepcopy

from collections import defaultdict
import os

cv_bridge = CvBridge()
class object_tracking:
    def __init__(self) -> None:
        rospy.init_node('object_tracking')
        rospy.on_shutdown(self.shutdown)
        rospy.logwarn("object_tracking node started")
        rospy.logwarn(f"current dir: {os.getcwd()}")
        self.pub_img_raw = rospy.Publisher('image_raw', Image, queue_size=10)
        self.pub_img_yolo = rospy.Publisher('image_yolo', Image, queue_size=10)
        # Load a pretrained YOLO model (recommended for training)
        self.model = YOLO('yolov8n.pt')
        self.names = self.model.model.names
        
        rospy.loginfo(f"Loaded YOLO model with {len(self.names)} classes")
        self.track_history = defaultdict(lambda: [])
        video_path = "src/1MinuteVideo-Doggie.mp4"
        self.cap = cv2.VideoCapture(video_path)
        assert self.cap.isOpened(), 'Cannot capture source'
        self.w, self.h, self.fps = (int(self.cap.get(x)) for x in (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT, cv2.CAP_PROP_FPS))        


    def shutdown(self):
        try:
            self.cap.release()
        except:
            pass
        rospy.loginfo("Shutting down object_tracking node")

    def track(self):
        while self.cap.isOpened():
            success, frame = self.cap.read()
            frame_copy = deepcopy(frame)
            if success:
                self.pub_img_raw.publish(cv_bridge.cv2_to_imgmsg(frame_copy, "bgr8"))
                results = self.model.track(frame, persist=True, verbose=False)
                boxes = results[0].boxes.xyxy

                if results[0].boxes.id is not None:

                    # Extract prediction results
                    clss = results[0].boxes.cls.tolist()
                    track_ids = results[0].boxes.id.int().tolist()
                    confs = results[0].boxes.conf.float().tolist()

                    # Annotator Init
                    annotator = Annotator(frame, line_width=2)

                    for box, cls, track_id in zip(boxes, clss, track_ids):
                        annotator.box_label(box, color=colors(int(cls), True), label=self.names[int(cls)])

                        # Store tracking history
                        track = self.track_history[track_id]
                        track.append((int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)))
                        if len(track) > 30:
                            track.pop(0)

                        # Plot tracks
                        points = np.array(track, dtype=np.int32).reshape((-1, 1, 2))
                        cv2.circle(frame, (track[-1]), 7, colors(int(cls), True), -1)
                        cv2.polylines(frame, [points], isClosed=False, color=colors(int(cls), True), thickness=2)
                
                # Publish image
                
                self.pub_img_yolo.publish(cv_bridge.cv2_to_imgmsg(frame, "bgr8"))
            else:
                break
        rospy.signal_shutdown('End of video')



if __name__ == '__main__':
    try:
        node = object_tracking()
        node.track()
    except rospy.ROSInterruptException:
        pass