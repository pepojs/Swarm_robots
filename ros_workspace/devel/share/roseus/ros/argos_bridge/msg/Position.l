;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::Position)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'Position (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::POSITION")
  (make-package "ARGOS_BRIDGE::POSITION"))

(in-package "ROS")
;;//! \htmlinclude Position.msg.html


(defclass argos_bridge::Position
  :super ros::object
  :slots (_position _orientation ))

(defmethod argos_bridge::Position
  (:init
   (&key
    ((:position __position) (instance argos_bridge::Vector3 :init))
    ((:orientation __orientation) (instance argos_bridge::Vector3 :init))
    )
   (send-super :init)
   (setq _position __position)
   (setq _orientation __orientation)
   self)
  (:position
   (&rest __position)
   (if (keywordp (car __position))
       (send* _position __position)
     (progn
       (if __position (setq _position (car __position)))
       _position)))
  (:orientation
   (&rest __orientation)
   (if (keywordp (car __orientation))
       (send* _orientation __orientation)
     (progn
       (if __orientation (setq _orientation (car __orientation)))
       _orientation)))
  (:serialization-length
   ()
   (+
    ;; argos_bridge/Vector3 _position
    (send _position :serialization-length)
    ;; argos_bridge/Vector3 _orientation
    (send _orientation :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; argos_bridge/Vector3 _position
       (send _position :serialize s)
     ;; argos_bridge/Vector3 _orientation
       (send _orientation :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; argos_bridge/Vector3 _position
     (send _position :deserialize buf ptr-) (incf ptr- (send _position :serialization-length))
   ;; argos_bridge/Vector3 _orientation
     (send _orientation :deserialize buf ptr-) (incf ptr- (send _orientation :serialization-length))
   ;;
   self)
  )

(setf (get argos_bridge::Position :md5sum-) "884731a1adb9599d204c6712c1265f2a")
(setf (get argos_bridge::Position :datatype-) "argos_bridge/Position")
(setf (get argos_bridge::Position :definition-)
      "Vector3 position
Vector3 orientation

================================================================================
MSG: argos_bridge/Vector3
float32 x
float32 y
float32 z

")



(provide :argos_bridge/Position "884731a1adb9599d204c6712c1265f2a")


