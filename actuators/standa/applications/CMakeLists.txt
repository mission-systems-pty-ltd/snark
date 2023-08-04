install( PROGRAMS standa-pan-tilt-control DESTINATION ${snark_INSTALL_BIN_DIR} COMPONENT Runtime )
install( FILES README.md config.json DESTINATION share/snark/actuators/standa COMPONENT Runtime )
