;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::Puck)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'Puck (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::PUCK")
  (make-package "ARGOS_BRIDGE::PUCK"))

(in-package "ROS")
;;//! \htmlinclude Puck.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass argos_bridge::Puck
  :super ros::object
  :slots (_color _range _angle ))

(defmethod argos_bridge::Puck
  (:init
   (&key
    ((:color __color) (instance std_msgs::ColorRGBA :init))
    ((:range __range) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _color __color)
   (setq _range (float __range))
   (setq _angle (float __angle))
   self)
  (:color
   (&rest __color)
   (if (keywordp (car __color))
       (send* _color __color)
     (progn
       (if __color (setq _color (car __color)))
       _color)))
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; std_msgs/ColorRGBA _color
    (send _color :serialization-length)
    ;; float32 _range
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/ColorRGBA _color
       (send _color :serialize s)
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/ColorRGBA _color
     (send _color :deserialize buf ptr-) (incf ptr- (send _color :serialization-length))
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get argos_bridge::Puck :md5sum-) "4ed42de6679ef0abec7e884aca1135c6")
(setf (get argos_bridge::Puck :datatype-) "argos_bridge/Puck")
(setf (get argos_bridge::Puck :definition-)
      "std_msgs/ColorRGBA color
float32 range
float32 angle

================================================================================
MSG: std_msgs/ColorRGBA
float32 r
float32 g
float32 b
float32 a

")



(provide :argos_bridge/Puck "4ed42de6679ef0abec7e884aca1135c6")


