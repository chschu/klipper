# Support for single-pin filament diameter sensors
#
# Copyright (C) 2024 Christian Schuster <christian@dnup.de>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from . import filament_switch_sensor

ADC_REPORT_TIME = 0.500
ADC_SAMPLE_TIME = 0.015
ADC_SAMPLE_COUNT = 32

class FilaSense:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.pin = config.get('pin')
        self.diameter_1 = config.getfloat('diameter_1', 1.5)
        self.diameter_2 = config.getfloat('diameter_2', 2.0)
        self.raw_1 = config.getint('raw_1', 6250)
        self.raw_2 = config.getint('raw_2', 8750)
        self.measurement_interval = config.getint('measurement_interval', 10)
        self.nominal_diameter = config.getfloat('nominal_diameter', above = 1.0)
        self.measurement_delay = config.getfloat('measurement_delay', above = 0.0)
        self.max_difference = config.getfloat('max_difference', 0.2)
        self.max_diameter = self.nominal_diameter + self.max_difference
        self.min_diameter = self.nominal_diameter - self.max_difference
        self.diameter = self.nominal_diameter
        self.enabled = config.getboolean('enabled', False)
        self.runout_min_diameter = config.getfloat('runout_min_diameter', 1.0)
        self.runout_max_diameter = config.getfloat('runout_max_diameter', self.max_diameter)
        self.logging = config.getboolean('logging', False)
        # use the current diameter instead of nominal while the first measurement isn't in place
        self.use_current_diameter_while_delay = config.getboolean('use_current_diameter_while_delay', False)
        # FIFO with tuples (epos, diameter)
        self.fifo = []
        self.raw = 0
        self.diameter = 0
        # printer objects
        self.toolhead = None
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # start ADC
        self.ppins = self.printer.lookup_object('pins')
        self.mcu_adc = self.ppins.setup_pin('adc', self.pin)
        self.mcu_adc.setup_minmax(ADC_SAMPLE_TIME, ADC_SAMPLE_COUNT)
        self.mcu_adc.setup_adc_callback(ADC_REPORT_TIME, self.adc_callback)
        # extrusion multiplier update timer
        self.timer = self.reactor.register_timer(self.timer_callback)
        # register commands
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('QUERY_FILAMENT_DIAMETER', self.cmd_QUERY_FILAMENT_DIAMETER)
        self.gcode.register_command('RESET_FILAMENT_DIAMETER_SENSOR', self.cmd_RESET_FILAMENT_DIAMETER_SENSOR)
        self.gcode.register_command('ENABLE_FILAMENT_DIAMETER_SENSOR', self.cmd_ENABLE_FILAMENT_DIAMETER_SENSOR)
        self.gcode.register_command('DISABLE_FILAMENT_DIAMETER_SENSOR', self.cmd_DISABLE_FILAMENT_DIAMETER_SENSOR)
        self.gcode.register_command('QUERY_RAW_FILAMENT_DIAMETER', self.cmd_QUERY_RAW_FILAMENT_DIAMETER)
        self.gcode.register_command('ENABLE_FILAMENT_DIAMETER_LOG', self.cmd_ENABLE_FILAMENT_DIAMETER_LOG)
        self.gcode.register_command('DISABLE_FILAMENT_DIAMETER_LOG',self.cmd_DISABLE_FILAMENT_DIAMETER_LOG)

        self.runout_helper = filament_switch_sensor.RunoutHelper(config)

    # klippy:ready callback
    def handle_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        # start extrusion multiplier update timer
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    # ADC callback
    def adc_callback(self, read_time, read_value):
        # read raw sensor value
        self.raw = round(read_value * 10000)
        # update diameter
        self.diameter = round((self.diameter_2 - self.diameter_1) / (self.raw_2 - self.raw_1) * (self.raw - self.raw_1) + self.diameter_1, 2)

    # extrusion multiplier update timer callback
    def timer_callback(self, eventtime):
        pos = self.toolhead.get_position()
        epos = pos[3]
        # determine projected extruder position when the current diameter will be active
        epos_projected = epos + self.measurement_delay
        # append projected extruder position and diameter to FIFO tail if measurement interval has passed or FIFO is empty
        if not self.fifo or epos_projected >= self.fifo[-1][0] + self.measurement_interval:
            self.fifo.append((epos_projected, self.diameter))
            if self.logging:
                self.gcode.respond_info("Filament diameter: %.2f" % self.diameter)
        # check runout
        filament_present = self.runout_min_diameter <= self.diameter <= self.runout_max_diameter;
        self.runout_helper.note_filament_present(filament_present)
        if filament_present:
            # check if extruder position from FIFO head has been reached; FIFO is never empty here
            if epos >= self.fifo[0][0]:
                # remove head entry from FIFO and use its diameter
                diameter_to_use = self.fifo.pop(0)[1]
            elif self.use_current_diameter_while_delay:
                # use current diameter in delay phase
                diameter_to_use = self.diameter
            else:
                # use nominal diameter in delay phase
                diameter_to_use = self.nominal_diameter
            # use nominal diameter if determined diameter is out of bounds
            if not self.min_diameter <= diameter_to_use <= self.max_diameter:
                diameter_to_use = self.nominal_diameter
            # update extrusion multiplier
            self.gcode.run_script("M221 S%d" % round((self.nominal_diameter / diameter_to_use) ** 2 * 100))
        else:
            # filament not present; set extrusion multiplier to 100% and clear FIFO
            self.gcode.run_script("M221 S100")
            self.fifo = []

        if self.enabled:
            return eventtime + 1
        else:
            return self.reactor.NEVER

    def cmd_QUERY_FILAMENT_DIAMETER(self, gcmd):
        if self.runout_min_diameter <= self.diameter <= self.runout_max_diameter:
            response = "Filament diameter: %.2f" % self.diameter
        else:
            response = "Filament NOT present"
        gcmd.respond_info(response)

    def cmd_RESET_FILAMENT_DIAMETER_SENSOR(self, gcmd):
        self.fifo = []
        gcmd.respond_info("Filament diameter measurements cleared!")
        # set extrusion multiplier to 100%
        self.gcode.run_script_from_command("M221 S100")

    def cmd_ENABLE_FILAMENT_DIAMETER_SENSOR(self, gcmd):
        if self.enabled:
            response = "Filament diameter sensor is already ON"
        else:
            response = "Filament diameter sensor turned ON"
            self.enabled = True
            # start extrusion multiplier update timer
            self.reactor.update_timer(self.timer, self.reactor.NOW)
        gcmd.respond_info(response)

    def cmd_DISABLE_FILAMENT_DIAMETER_SENSOR(self, gcmd):
        if not self.enabled:
            response = "Filament diameter sensor is already OFF"
        else:
            response = "Filament diameter sensor turned OFF"
            self.enabled = False
            # stop extrusion multiplier update timer
            self.reactor.update_timer(self.timer, self.reactor.NEVER)
            # clear FIFO
            self.fifo = []
            # set extrusion multiplier to 100%
            self.gcode.run_script_from_command("M221 S100")
        gcmd.respond_info(response)

    def cmd_QUERY_RAW_FILAMENT_DIAMETER(self, gcmd):
        gcmd.respond_info("RAW=%d" % self.raw)

    def get_status(self, eventtime):
        return { 'diameter': self.diameter, 'raw': self.raw, 'enabled': self.enabled }

    def cmd_ENABLE_FILAMENT_DIAMETER_LOG(self, gcmd):
        self.logging = True
        gcmd.respond_info("Filament diameter logging turned ON")

    def cmd_DISABLE_FILAMENT_DIAMETER_LOG(self, gcmd):
        self.logging = False
        gcmd.respond_info("Filament diameter logging turned OFF")

def load_config(config):
    return FilaSense(config)
