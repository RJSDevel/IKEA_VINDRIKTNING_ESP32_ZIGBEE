const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const extend = require('zigbee-herdsman-converters/lib/extend');
const constants = require('zigbee-herdsman-converters/lib/constants');
const {postfixWithEndpointName} = require('zigbee-herdsman-converters/lib/utils');
const e = exposes.presets;
const ea = exposes.access;

const fromZigbee = {
	pm25: {
        cluster: 'pm25Measurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
			if (msg.data.hasOwnProperty('measuredValue')) {
				const ep = postfixWithEndpointName('pm25', msg, model, meta);
				return {[ep]: parseFloat(msg.data.measuredValue)};
			}
        },
    }
}

const definition = {
        zigbeeModel: ['VINDRIKTNING_YNXT'],
        model: 'VINDRIKTNING_YNXT',
        vendor: 'IKEA',
        description: '[IKEA VINDRIKTNING - PM2.5, PM10, PM1 sensor]',
        fromZigbee: [fromZigbee.pm25, fz.fan],
        toZigbee: [tz.fan_mode],
		meta: {
			multiEndpoint: true
		},
		endpoint: (device) => {
			return {
				'l1': 1,
				'l2': 2,
				'l3': 3,
			};
		},
        configure: async (device, coordinatorEndpoint, logger) => {
            const endpoint25 = device.getEndpoint(1);
			await endpoint25.bind('pm25Measurement', constants.defaultBindGroup);
			await reporting.bind(endpoint25, coordinatorEndpoint, ['pm25Measurement']);
			await reporting.bind(endpoint25, coordinatorEndpoint, ['hvacFanCtrl']);
			await reporting.fanMode(endpoint25);
			
			const endpoint1 = device.getEndpoint(2);
			await endpoint1.bind('pm25Measurement', constants.defaultBindGroup);
			await reporting.bind(endpoint1, coordinatorEndpoint, ['pm25Measurement']);
			const endpoint10 = device.getEndpoint(3);
			await endpoint10.bind('pm25Measurement', constants.defaultBindGroup);
			await reporting.bind(endpoint10, coordinatorEndpoint, ['pm25Measurement']);
        },
		exposes:[
			e.fan().withModes(['low', 'medium', 'high', 'on', 'auto']),
			exposes.numeric('pm25', ea.STATE).withUnit('μg/m³').withDescription('PM2.5').withEndpoint('l1'),
			exposes.numeric('pm25', ea.STATE).withUnit('μg/m³').withDescription('PM1').withEndpoint('l2'),
			exposes.numeric('pm25', ea.STATE).withUnit('μg/m³').withDescription('PM10').withEndpoint('l3'),
		]
    
};

module.exports = definition;