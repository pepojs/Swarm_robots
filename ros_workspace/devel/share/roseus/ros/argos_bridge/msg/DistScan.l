;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::DistScan)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'DistScan (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::DISTSCAN")
  (make-package "ARGOS_BRIDGE::DISTSCAN"))

(in-package "ROS")
;;//! \htmlinclude DistScan.msg.html


(defclass argos_bridge::DistScan
  :super ros::object
  :slots (_range _angle ))

(defmethod argos_bridge::DistScan
  (:init
   (&key
    ((:range __range) 0.0)
    ((:angle __angle) 0.0)
    )
   (send-super :init)
   (setq _range (float __range))
   (setq _angle (float __angle))
   self)
  (:range
   (&optional __range)
   (if __range (setq _range __range)) _range)
  (:angle
   (&optional __angle)
   (if __angle (setq _angle __angle)) _angle)
  (:serialization-length
   ()
   (+
    ;; float32 _range
    4
    ;; float32 _angle
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _range
       (sys::poke _range (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _angle
       (sys::poke _angle (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _range
     (setq _range (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _angle
     (setq _angle (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get argos_bridge::DistScan :md5sum-) "5d665e328703e395c9fbb21d2decd1b5")
(setf (get argos_bridge::DistScan :datatype-) "argos_bridge/DistScan")
(setf (get argos_bridge::DistScan :definition-)
      "float32 range
float32 angle
")



(provide :argos_bridge/DistScan "5d665e328703e395c9fbb21d2decd1b5")


