; Auto-generated. Do not edit!


(cl:in-package argos_bridge-msg)


;//! \htmlinclude MotoGroundList.msg.html

(cl:defclass <MotoGroundList> (roslisp-msg-protocol:ros-message)
  ((n
    :reader n
    :initarg :n
    :type cl:integer
    :initform 0)
   (motoGrounds
    :reader motoGrounds
    :initarg :motoGrounds
    :type (cl:vector argos_bridge-msg:MotoGround)
   :initform (cl:make-array 0 :element-type 'argos_bridge-msg:MotoGround :initial-element (cl:make-instance 'argos_bridge-msg:MotoGround))))
)

(cl:defclass MotoGroundList (<MotoGroundList>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <MotoGroundList>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'MotoGroundList)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name argos_bridge-msg:<MotoGroundList> is deprecated: use argos_bridge-msg:MotoGroundList instead.")))

(cl:ensure-generic-function 'n-val :lambda-list '(m))
(cl:defmethod n-val ((m <MotoGroundList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:n-val is deprecated.  Use argos_bridge-msg:n instead.")
  (n m))

(cl:ensure-generic-function 'motoGrounds-val :lambda-list '(m))
(cl:defmethod motoGrounds-val ((m <MotoGroundList>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader argos_bridge-msg:motoGrounds-val is deprecated.  Use argos_bridge-msg:motoGrounds instead.")
  (motoGrounds m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <MotoGroundList>) ostream)
  "Serializes a message object of type '<MotoGroundList>"
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n)) ostream)
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'motoGrounds))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'motoGrounds))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <MotoGroundList>) istream)
  "Deserializes a message object of type '<MotoGroundList>"
    (cl:setf (cl:ldb (cl:byte 8 0) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) (cl:slot-value msg 'n)) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) (cl:slot-value msg 'n)) (cl:read-byte istream))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'motoGrounds) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'motoGrounds)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'argos_bridge-msg:MotoGround))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<MotoGroundList>)))
  "Returns string type for a message object of type '<MotoGroundList>"
  "argos_bridge/MotoGroundList")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'MotoGroundList)))
  "Returns string type for a message object of type 'MotoGroundList"
  "argos_bridge/MotoGroundList")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<MotoGroundList>)))
  "Returns md5sum for a message object of type '<MotoGroundList>"
  "49d994c0a5d5dc1356fd2c48d2b519a1")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'MotoGroundList)))
  "Returns md5sum for a message object of type 'MotoGroundList"
  "49d994c0a5d5dc1356fd2c48d2b519a1")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<MotoGroundList>)))
  "Returns full string definition for message of type '<MotoGroundList>"
  (cl:format cl:nil "uint32 n~%MotoGround[] motoGrounds~%~%================================================================================~%MSG: argos_bridge/MotoGround~%float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'MotoGroundList)))
  "Returns full string definition for message of type 'MotoGroundList"
  (cl:format cl:nil "uint32 n~%MotoGround[] motoGrounds~%~%================================================================================~%MSG: argos_bridge/MotoGround~%float32 value~%float32 offset_x~%float32 offset_y~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <MotoGroundList>))
  (cl:+ 0
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'motoGrounds) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <MotoGroundList>))
  "Converts a ROS message object to a list"
  (cl:list 'MotoGroundList
    (cl:cons ':n (n msg))
    (cl:cons ':motoGrounds (motoGrounds msg))
))
