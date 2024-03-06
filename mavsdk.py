import asyncio

from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)


async def run():

    drone = System()
    await drone.connect(system_address="udp://:14540")
    
    #status_text_task = asyncio.ensure_future(print_status_text(drone))

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break        
            
    async for POSCTL in drone.telemetry.flight_mode(): #11 = POSCTL mode = POSHOLD mission planner
        if POSCTL:
            print("flight mode:", POSCTL)
            break 
        	
    
    #callibration first
    #print("-- Starting gyroscope calibration")
    #async for progress_data in drone.calibration.calibrate_gyro():
    #    print(progress_data)
    #print("-- Gyroscope calibration finished")

#    print("-- Starting accelerometer calibration")
 #   async for progress_data in drone.calibration.calibrate_accelerometer():
  #      print(progress_data)
  #  print("-- Accelerometer calibration finished")

#    print("-- Starting magnetometer calibration")
 #   async for progress_data in drone.calibration.calibrate_magnetometer():
  #      print(progress_data)
   # print("-- Magnetometer calibration finished")

    #print("-- Starting board level horizon calibration")
    #async for progress_data in drone.calibration.calibrate_level_horizon():
   #     print(progress_data)
   # print("-- Board level calibration finished")


 

    print("-- Arming")
    await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    print("-- Go 0m North, 0m East, -5m Down \
            within local coordinate system")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -3.0, 0.0))
    await asyncio.sleep(10)

    print("-- Go 5m North, 0m East, -5m Down \
            within local coordinate system, turn to face East")
    await drone.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, -3.0, 90.0))
    await asyncio.sleep(10)


    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                with error code: {error._result.result}")



    print("-- Landing")
    await drone.action.land()

    #status_text_task.cancel()


#async def print_status_text(drone):
    #try:
        #async for status_text in drone.telemetry.status_text():
            #print(f"Status: {status_text.type}: {status_text.text}")
    #except asyncio.CancelledError:
        #return


if __name__ == "__main__":
    # Run the asyncio loop
    asyncio.run(run())
