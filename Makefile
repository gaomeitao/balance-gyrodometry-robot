all:	
	@make -C robocape
	@make -C lcmtypes
	@make -C balancebot
	@make -C imu_lcm_example
	@make -C optitrack/common
	@make -C optitrack

groundstation:
	@make -C java
	@make -C lcmtypes
	@make -C optitrack/common
	@make -C optitrack
	@make -C botgui

clean:
	@make -C robocape -s clean
	@make -C lcmtypes -s clean
	@make -C balancebot -s clean
	@make -C imu_lcm_example -s clean
	@make -C java -s clean
	@make -C optitrack/common -s clean
	@make -C optitrack -s clean
	@make -C botgui -s clean
