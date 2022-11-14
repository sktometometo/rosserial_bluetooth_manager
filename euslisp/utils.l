(ros::load-ros-manifest "rosserial_bluetooth_manager")

(defun connect-device (bt-addr &key (server-name "/rosserial_bluetooth_manager/connect_device"))
  (let (res
        (req (instance rosserial_bluetooth_manager::ConnectDeviceRequest
                       :init
                       :address bt-addr)))
    (setq res (ros::service-call server-name req))
    (if (send res :success)
      (ros::ros-info (send res :message))
      (ros::ros-error (send res :message))
      )
    res
    )
  )

(defun disconnect-device (bt-addr &key (server-name "/rosserial_bluetooth_manager/disconnect_device"))
  (let (res
        (req (instance rosserial_bluetooth_manager::DisconnectDeviceRequest
                       :init
                       :address bt-addr)))
    (setq res (ros::service-call server-name req))
    (if (send res :success)
      (ros::ros-info (send res :message))
      (ros::ros-error (send res :message))
      )
    res
    )
  )
