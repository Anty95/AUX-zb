/**
 * Zigbee2MQTT External Converter for H-Link HVAC Controller
 * 
 * This version includes polling for attributes that ESP-Zigbee stack cannot auto-report.
 * 
 * REPORTING FLAG OPTIMIZATION:
 * - Device uses ESP_ZB_ZCL_ATTR_ACCESS_REPORTING flag for automatic attribute reporting
 * - Temperature, setpoints, system_mode, and all switches auto-report on changes
 * - Minimal polling only for attributes ESP-Zigbee stack cannot auto-report:
 *   * runningState (Zigbee stack limitation)
 *   * fanMode (not in standard reportable attributes)
 *   * outdoorTemp (not typically reportable)
 * 
 * Endpoints:
 * 1: Main thermostat (HVAC + fan control)
 * 2: Swing mode switch
 * 3: Remote control lock switch
 * 4: Beeper switch
 * 5: Leave home (away preset) switch
 * 6: Air filter warning binary sensor (read-only)
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const m = require('zigbee-herdsman-converters/lib/modernExtend');
const e = exposes.presets;
const ea = exposes.access;

// Custom converters
const fzLocal = {
    thermostat: {
        cluster: 'hvacThermostat',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg) => {
            const res = {};
            if (msg.data.localTemp !== undefined) {
                res.local_temperature = msg.data.localTemp / 100;
            }
            if (msg.data.systemMode !== undefined) {
                const map = {0x00: 'off', 0x01: 'auto', 0x03: 'cool', 0x04: 'heat', 0x07: 'fan_only', 0x08: 'dry'};
                res.system_mode = map[msg.data.systemMode] || 'off';
            }
            if (msg.data.occupiedHeatingSetpoint !== undefined) {
                res.occupied_heating_setpoint = msg.data.occupiedHeatingSetpoint / 100;
            }
            if (msg.data.runningState !== undefined) {
                const map = {0x0000: 'idle', 0x0001: 'heat', 0x0002: 'cool', 0x0004: 'fan_only', 0x0008: 'dry'};
                res.running_state = map[msg.data.runningState] || 'idle';
            }
            if (msg.data.outdoorTemp !== undefined) {
                res.outdoor_temperature = msg.data.outdoorTemp / 100;
            }
            return res;
        },
    },
    fan_mode: {
        cluster: 'hvacFanCtrl',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg) => {
            if (msg.data.fanMode !== undefined) {
                const map = {0x00: 'auto', 0x01: 'high', 0x02: 'medium', 0x03: 'low', 0x04: 'quiet'};
                return {fan_mode: map[msg.data.fanMode] || 'auto'};
            }
        },
    },
    filter_warning: {
        cluster: 'genBinaryInput',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg) => {
            if (msg.endpoint.ID === 6 && msg.data.presentValue !== undefined) {
                return {filter_warning: msg.data.presentValue === 1 ? 'ON' : 'OFF'};
            }
        },
    },
};

const tzLocal = {
    fan_mode: {
        key: ['fan_mode'],
        convertSet: async (entity, key, value) => {
            const map = {'auto': 0x00, 'high': 0x01, 'medium': 0x02, 'low': 0x03, 'quiet': 0x04};
            const mode = map[(value || '').toLowerCase()];
            if (mode === undefined) throw new Error(`Unsupported fan_mode ${value}`);
            await entity.write('hvacFanCtrl', {fanMode: mode});
            return {state: {fan_mode: value}};
        },
        convertGet: async (entity) => {
            await entity.read('hvacFanCtrl', ['fanMode']);
        },
    },
};

const definition = {
    zigbeeModel: ['hlink-zb'],
    model: 'H-Link-ZB',
    vendor: 'Custom devices (DiY)',
    description: 'H-Link Zigbee Bridge for Hitachi HVAC (Router)',
    meta: {multiEndpoint: true},

    fromZigbee: [
        fzLocal.thermostat,
        fzLocal.fan_mode,
        fzLocal.filter_warning,
        fz.on_off, // switch endpoints 2-5
    ],
    toZigbee: [
        tz.thermostat_local_temperature,
        tz.thermostat_occupied_heating_setpoint,
        tz.thermostat_system_mode,
        tzLocal.fan_mode,
        tz.on_off, // switch endpoints 2-5
    ],

    exposes: [
        e.climate()
            .withSetpoint('occupied_heating_setpoint', 16, 32, 1)
            .withLocalTemperature()
            .withSystemMode(['off', 'auto', 'cool', 'heat', 'dry', 'fan_only'])
            .withRunningState(['idle', 'heat', 'cool', 'fan_only'])
            .withDescription('Main thermostat'),
        exposes.enum('fan_mode', exposes.access.ALL, ['auto', 'low', 'medium', 'high', 'quiet'])
            .withDescription('Fan speed control'),
        exposes.numeric('outdoor_temperature', exposes.access.STATE_GET)
            .withUnit('°C')
            .withDescription('Outdoor temperature (read-only)'),
        e.switch().withEndpoint('swing_mode').withDescription('Swing mode'),
        e.switch().withEndpoint('remote_lock').withDescription('Remote control lock'),
        e.switch().withEndpoint('beeper').withDescription('Beeper'),
        e.switch().withEndpoint('leave_home').withDescription('Leave home / away preset'),
        exposes.binary('filter_warning', exposes.access.STATE_GET, 'ON', 'OFF')
            .withDescription('Air filter needs cleaning (read-only)'),
    ],

    endpoint: (device) => ({
        default: 1,
        swing_mode: 2,
        remote_lock: 3,
        beeper: 4,
        leave_home: 5,
        filter_warning: 6,
    }),

    configure: async (device, coordinatorEndpoint, logger) => {
        const ep1 = device.getEndpoint(1);
        const ep2 = device.getEndpoint(2);
        const ep3 = device.getEndpoint(3);
        const ep4 = device.getEndpoint(4);
        const ep5 = device.getEndpoint(5);
        const ep6 = device.getEndpoint(6);

        // Bind clusters for main thermostat (endpoint 1)
        await reporting.bind(ep1, coordinatorEndpoint, ['genBasic', 'hvacThermostat', 'hvacFanCtrl']);
        
        // Configure reporting for thermostat attributes (with REPORTING flag, these auto-report!)
        await reporting.thermostatTemperature(ep1);  // localTemp
        await reporting.thermostatOccupiedHeatingSetpoint(ep1);  // setpoint
        
        // Configure systemMode reporting (0x001C)
        await ep1.configureReporting('hvacThermostat', [{
            attribute: 'systemMode',
            minimumReportInterval: 1,
            maximumReportInterval: 300,
            reportableChange: 1,
        }]);
        
        // Note: runningState (0x0029) is NOT auto-reportable by ESP-Zigbee stack - we poll it
        
        // Note: fanMode is NOT auto-reportable by ESP-Zigbee stack - we poll it
        
        // Configure outdoor temperature reporting (if available)
        try {
            await ep1.configureReporting('hvacThermostat', [{
                attribute: 'outdoorTemp',
                minimumReportInterval: 60,
                maximumReportInterval: 3600,
                reportableChange: 50,  // 0.5°C
            }]);
        } catch (err) {
            logger.debug(`H-Link: outdoorTemp reporting not available: ${err.message}`);
        }

        // Bind and configure on/off switches (endpoints 2-5) - with REPORTING flag, these auto-report!
        for (const ep of [ep2, ep3, ep4, ep5]) {
            if (ep) {
                await reporting.bind(ep, coordinatorEndpoint, ['genOnOff']);
                await reporting.onOff(ep);
            }
        }

        // Bind and configure filter warning sensor (endpoint 6)
        if (ep6) {
            await reporting.bind(ep6, coordinatorEndpoint, ['genBinaryInput']);
            await ep6.configureReporting('genBinaryInput', [{
                attribute: 'presentValue',
                minimumReportInterval: 60,
                maximumReportInterval: 3600,
                reportableChange: 1,
            }]);
        }

        // Initial read of unreportable attributes (only the ones we need to poll)
        try {
            await ep1.read('hvacThermostat', ['runningState']);  // Not auto-reportable
            await ep1.read('hvacFanCtrl', ['fanMode']);  // Not auto-reportable
            try { 
                await ep1.read('hvacThermostat', ['outdoorTemp']);  // Not typically reportable
            } catch (_) {}
            if (ep6) await ep6.read('genBinaryInput', ['presentValue']);
        } catch (err) {
            logger.warn(`H-Link configure: initial read failed: ${err.message}`);
        }
    },

    // Minimal polling for truly unreportable attributes only (with REPORTING flag, most attributes now auto-report!)
    extend: [
        m.poll({
            key: "hlink_state",
            option: e
                .numeric("hlink_poll_interval", ea.SET)
                .withValueMin(-1)
                .withDescription(
                    "H-Link HVAC minimal polling for attributes that ESP-Zigbee stack cannot auto-report (runningState, fanMode, outdoorTemp). Default is 60 seconds. Most attributes now auto-report via REPORTING flag! Set to -1 to disable."
                ),
            defaultIntervalSeconds: 60,  // Can be increased since most things now auto-report
            poll: async (device) => {
                const endpoint1 = device.getEndpoint(1);
                if (!endpoint1) {
                    console.warn(`H-Link polling: endpoint 1 not found`);
                    return;
                }
                
                // Poll ONLY the truly unreportable attributes
                try {
                    // runningState - ESP-Zigbee stack limitation, cannot auto-report
                    await endpoint1.read('hvacThermostat', ['runningState']);
                } catch (error) {
                    console.error(`H-Link polling: runningState read failed: ${error.message}`);
                }
                
                try {
                    // fanMode - Not in standard reportable attributes
                    await endpoint1.read('hvacFanCtrl', ['fanMode']);
                } catch (error) {
                    console.error(`H-Link polling: fanMode read failed: ${error.message}`);
                }
                
                try {
                    // outdoorTemp - Not typically reportable
                    await endpoint1.read('hvacThermostat', ['outdoorTemp']);
                } catch (error) {
                    console.error(`H-Link polling: outdoorTemp read failed: ${error.message}`);
                }
            },
        }),
    ],
    ota: true,
};

module.exports = definition;
