void OdriveInit() {

      for (int axis = 0; axis < 2; ++axis) {
          Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 120000.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.current_lim " << 40.0f << '\n';
          Serial1 << "w axis" << axis << ".motor.config.calibration_current " << 10.0f << '\n';

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, false); // don't wait 

          delay(1000);
      }
            
         
  
}



