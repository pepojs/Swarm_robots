;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::MotoGround)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'MotoGround (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::MOTOGROUND")
  (make-package "ARGOS_BRIDGE::MOTOGROUND"))

(in-package "ROS")
;;//! \htmlinclude MotoGround.msg.html


(defclass argos_bridge::MotoGround
  :super ros::object
  :slots (_value _offset_x _offset_y ))

(defmethod argos_bridge::MotoGround
  (:init
   (&key
    ((:value __value) 0.0)
    ((:offset_x __offset_x) 0.0)
    ((:offset_y __offset_y) 0.0)
    )
   (send-super :init)
   (setq _value (float __value))
   (setq _offset_x (float __offset_x))
   (setq _offset_y (float __offset_y))
   self)
  (:value
   (&optional __value)
   (if __value (setq _value __value)) _value)
  (:offset_x
   (&optional __offset_x)
   (if __offset_x (setq _offset_x __offset_x)) _offset_x)
  (:offset_y
   (&optional __offset_y)
   (if __offset_y (setq _offset_y __offset_y)) _offset_y)
  (:serialization-length
   ()
   (+
    ;; float32 _value
    4
    ;; float32 _offset_x
    4
    ;; float32 _offset_y
    4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; float32 _value
       (sys::poke _value (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset_x
       (sys::poke _offset_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _offset_y
       (sys::poke _offset_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; float32 _value
     (setq _value (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset_x
     (setq _offset_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _offset_y
     (setq _offset_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;;
   self)
  )

(setf (get argos_bridge::MotoGround :md5sum-) "df359e3890070432de3f37977284cdda")
(setf (get argos_bridge::MotoGround :datatype-) "argos_bridge/MotoGround")
(setf (get argos_bridge::MotoGround :definition-)
      "float32 value
float32 offset_x
float32 offset_y

")



(provide :argos_bridge/MotoGround "df359e3890070432de3f37977284cdda")


