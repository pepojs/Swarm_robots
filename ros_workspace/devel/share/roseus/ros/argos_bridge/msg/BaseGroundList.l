;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::BaseGroundList)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'BaseGroundList (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::BASEGROUNDLIST")
  (make-package "ARGOS_BRIDGE::BASEGROUNDLIST"))

(in-package "ROS")
;;//! \htmlinclude BaseGroundList.msg.html


(defclass argos_bridge::BaseGroundList
  :super ros::object
  :slots (_n _baseGrounds ))

(defmethod argos_bridge::BaseGroundList
  (:init
   (&key
    ((:n __n) 0)
    ((:baseGrounds __baseGrounds) (let (r) (dotimes (i 0) (push (instance argos_bridge::BaseGround :init) r)) r))
    )
   (send-super :init)
   (setq _n (round __n))
   (setq _baseGrounds __baseGrounds)
   self)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:baseGrounds
   (&rest __baseGrounds)
   (if (keywordp (car __baseGrounds))
       (send* _baseGrounds __baseGrounds)
     (progn
       (if __baseGrounds (setq _baseGrounds (car __baseGrounds)))
       _baseGrounds)))
  (:serialization-length
   ()
   (+
    ;; uint32 _n
    4
    ;; argos_bridge/BaseGround[] _baseGrounds
    (apply #'+ (send-all _baseGrounds :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint32 _n
       (write-long _n s)
     ;; argos_bridge/BaseGround[] _baseGrounds
     (write-long (length _baseGrounds) s)
     (dolist (elem _baseGrounds)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; argos_bridge/BaseGround[] _baseGrounds
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _baseGrounds (let (r) (dotimes (i n) (push (instance argos_bridge::BaseGround :init) r)) r))
     (dolist (elem- _baseGrounds)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argos_bridge::BaseGroundList :md5sum-) "69c1c089ae74b076579a8bd75e220110")
(setf (get argos_bridge::BaseGroundList :datatype-) "argos_bridge/BaseGroundList")
(setf (get argos_bridge::BaseGroundList :definition-)
      "uint32 n
BaseGround[] baseGrounds

================================================================================
MSG: argos_bridge/BaseGround
float32 value
float32 offset_x
float32 offset_y

")



(provide :argos_bridge/BaseGroundList "69c1c089ae74b076579a8bd75e220110")


