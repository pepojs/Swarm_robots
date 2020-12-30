;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::MotoGroundList)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'MotoGroundList (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::MOTOGROUNDLIST")
  (make-package "ARGOS_BRIDGE::MOTOGROUNDLIST"))

(in-package "ROS")
;;//! \htmlinclude MotoGroundList.msg.html


(defclass argos_bridge::MotoGroundList
  :super ros::object
  :slots (_n _motoGrounds ))

(defmethod argos_bridge::MotoGroundList
  (:init
   (&key
    ((:n __n) 0)
    ((:motoGrounds __motoGrounds) (let (r) (dotimes (i 0) (push (instance argos_bridge::MotoGround :init) r)) r))
    )
   (send-super :init)
   (setq _n (round __n))
   (setq _motoGrounds __motoGrounds)
   self)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:motoGrounds
   (&rest __motoGrounds)
   (if (keywordp (car __motoGrounds))
       (send* _motoGrounds __motoGrounds)
     (progn
       (if __motoGrounds (setq _motoGrounds (car __motoGrounds)))
       _motoGrounds)))
  (:serialization-length
   ()
   (+
    ;; uint32 _n
    4
    ;; argos_bridge/MotoGround[] _motoGrounds
    (apply #'+ (send-all _motoGrounds :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _n
       (write-long _n s)
     ;; argos_bridge/MotoGround[] _motoGrounds
     (write-long (length _motoGrounds) s)
     (dolist (elem _motoGrounds)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; argos_bridge/MotoGround[] _motoGrounds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _motoGrounds (let (r) (dotimes (i n) (push (instance argos_bridge::MotoGround :init) r)) r))
     (dolist (elem- _motoGrounds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argos_bridge::MotoGroundList :md5sum-) "49d994c0a5d5dc1356fd2c48d2b519a1")
(setf (get argos_bridge::MotoGroundList :datatype-) "argos_bridge/MotoGroundList")
(setf (get argos_bridge::MotoGroundList :definition-)
      "uint32 n
MotoGround[] motoGrounds

================================================================================
MSG: argos_bridge/MotoGround
float32 value
float32 offset_x
float32 offset_y

")



(provide :argos_bridge/MotoGroundList "49d994c0a5d5dc1356fd2c48d2b519a1")


