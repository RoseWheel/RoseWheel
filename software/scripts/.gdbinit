# Set breakpoint parameters for Cortex-M3
set remote hardware-breakpoint-limit 6
set remote hardware-watchpoint-limit 4

# Disable IRQs while stepping 
define hook-step
  mon cortex_m3 maskisr on
end
define hookpost-step
  mon cortex_m3 maskisr off
end

# Connect to openocd on localhost
define connect
  target remote localhost:3333
end

# Flash the program
define flash
  monitor halt; stm32x unlock 0; reset halt; flash write_image erase main; reset halt
  load
end

# Flash the program and run
define flash-and-run
  flash
  continue
end
