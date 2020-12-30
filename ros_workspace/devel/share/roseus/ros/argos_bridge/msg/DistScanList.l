;; Auto-generated. Do not edit!


(when (boundp 'argos_bridge::DistScanList)
  (if (not (find-package "ARGOS_BRIDGE"))
    (make-package "ARGOS_BRIDGE"))
  (shadow 'DistScanList (find-package "ARGOS_BRIDGE")))
(unless (find-package "ARGOS_BRIDGE::DISTSCANLIST")
  (make-package "ARGOS_BRIDGE::DISTSCANLIST"))

(in-package "ROS")
;;//! \htmlinclude DistScanList.msg.html


(defclass argos_bridge::DistScanList
  :super ros::object
  :slots (_n _scan ))

(defmethod argos_bridge::DistScanList
  (:init
   (&key
    ((:n __n) 0)
    ((:scan __scan) (let (r) (dotimes (i 0) (push (instance argos_bridge::DistScan :init) r)) r))
    )
   (send-super :init)
   (setq _n (round __n))
   (setq _scan __scan)
   self)
  (:n
   (&optional __n)
   (if __n (setq _n __n)) _n)
  (:scan
   (&rest __scan)
   (if (keywordp (car __scan))
       (send* _scan __scan)
     (progn
       (if __scan (setq _scan (car __scan)))
       _scan)))
  (:serialization-length
   ()
   (+
    ;; int32 _n
    4
    ;; argos_bridge/DistScan[] _scan
    (apply #'+ (send-all _scan :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _n
       (write-long _n s)
     ;; argos_bridge/DistScan[] _scan
     (write-long (length _scan) s)
     (dolist (elem _scan)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _n
     (setq _n (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; argos_bridge/DistScan[] _scan
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _scan (let (r) (dotimes (i n) (push (instance argos_bridge::DistScan :init) r)) r))
     (dolist (elem- _scan)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get argos_bridge::DistScanList :md5sum-) "7fb89b86c4713c9df9a68e4b0fd602da")
(setf (get argos_bridge::DistScanList :datatype-) "argos_bridge/DistScanList")
(setf (get argos_bridge::DistScanList :definition-)
      "int32 n
DistScan[] scan
================================================================================
MSG: argos_bridge/DistScan
float32 range
float32 angle
")



(provide :argos_bridge/DistScanList "7fb89b86c4713c9df9a68e4b0fd602da")


