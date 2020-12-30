; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude DistScanList.msg.html

(cl:defclass <DistScanList> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (scan
    :reader scan
    :initarg :scan
    :type (cl:vector argos_bridge-msg:DistScan)
   :initform (cl:make-array 0 :element-type 'argos_bridge-msg:DistScan :initial-element (cl:make-instance 'argos_bridge-msg:DistScan))))
)

(cl:defclass DistScanList (<DistScanList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <DistScanList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'DistScanList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<DistScanList> is deprecated: use argos_bridge-msg:DistScanList instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <DistScanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:n-val is deprecated.  Use argos_bridge-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'scan-val :lambda-list '(m))
(cl:defmethod scan-val ((m <DistScanList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:scan-val is deprecated.  Use argos_bridge-msg:scan instead.")
  (scan m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <DistScanList>) ostream)
  "Serializes a message object of type '<DistScanList>"
  (cl:let* ((signed (cl:slot-value msg 'n)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'scan))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'scan))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <DistScanList>) istream)
  "Deserializes a message object of type '<DistScanList>"
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'n) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'scan) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'scan)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argos_bridge-msg:DistScan))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<DistScanList>)))
  "Returns string type for a message object of type '<DistScanList>"
  "argos_bridge/DistScanList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'DistScanList)))
  "Returns string type for a message object of type 'DistScanList"
  "argos_bridge/DistScanList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<DistScanList>)))
  "Returns md5sum for a message object of type '<DistScanList>"
  "7fb89b86c4713c9df9a68e4b0fd602da")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'DistScanList)))
  "Returns md5sum for a message object of type 'DistScanList"
  "7fb89b86c4713c9df9a68e4b0fd602da")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<DistScanList>)))
  "Returns full string definition for message of type '<DistScanList>"
  (cl:format cl:nil "int32 n~%DistScan[] scan~%================================================================================~%MSG: argos_bridge/DistScan~%float32 range~%float32 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'DistScanList)))
  "Returns full string definition for message of type 'DistScanList"
  (cl:format cl:nil "int32 n~%DistScan[] scan~%================================================================================~%MSG: argos_bridge/DistScan~%float32 range~%float32 angle~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <DistScanList>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'scan) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <DistScanList>))
  "Converts a ROS message object to a list"
  (cl:list 'DistScanList
    (cl:cons ':n (n msg))
    (cl:cons ':scan (scan msg))
))
