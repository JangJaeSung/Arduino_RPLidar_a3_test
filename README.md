# Arduino_RPLidar_a3_test

<First>
Modify : Arduino RPLidar a3 header and cpp file (libraries)
```diff
diff --git a/rplidar.cpp b/rplidar.cpp
index 4b2d602..ced05eb 100644
--- a/rplidar.cpp
+++ b/rplidar.cpp
@@ -78,7 +78,7 @@ enum Byte {StartByte1=0xA5, StartByte2=0x5A, SyncBits1=0xA, SyncBits2=0x5};
 enum Offsets {OffsetDescriptorDataLengthSendMode = 2, OffsetDescriptorDataType = 6,
 			OffsetMeasurementCapsuledUltraCrc = 2, OffsetMeasurementCapsuledStartAngleSyncQ6=2};
 
-enum Masks : uint32_t {MaskResponseDescriptorDataLength=0x3FFFFFFF, ShiftResponseDescriptorSendMode=30, MaskNewScan=1 << 15};
+enum Masks : uint32_t {MaskResponseDescriptorDataLength=0x3FFFFFFF, ShiftResponseDescriptorSendMode=30, MaskNewScan=(uint32_t)1 << 15};
 
 enum {SecondsPerMinute=60, MicrosPerSecond=1000000};
 
@@ -113,7 +113,7 @@ RPLidar::RPLidar(HardwareSerial &serial, int pwm_pin):
 void RPLidar::setup(int motor_rpm)
 {
   pinMode(m_pwm_pin, OUTPUT);
-  analogWriteFrequency(m_pwm_pin, PwmFrequency);
+//  analogWriteFrequency(m_pwm_pin, PwmFrequency);
   m_serial.begin(UartBaudrate, SERIAL_8N1);
   m_motor_setpoint_rpm=motor_rpm;
   m_motor_actual_rpm=motor_rpm;
diff --git a/rplidar.h b/rplidar.h
index edd5dd5..a0cfc06 100644
--- a/rplidar.h
+++ b/rplidar.h
@@ -87,7 +87,7 @@ private:
 	}
 	inline uint32_t decode_u32(const uint8_t *data) const
 	{
-		return (uint32_t) data[3] << 24 | data[2] << 16  | data[1] << 8 | data[0];
+		return (uint32_t) data[3] << 24 | (uint32_t)data[2] << 16  | data[1] << 8 | data[0];
 	}
 private:
 	/* Hardware */
```
