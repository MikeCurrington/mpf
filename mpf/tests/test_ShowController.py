import time

from mock import MagicMock
from ruamel import yaml

from mpf.assets.show import Show
from mpf.core.rgb_color import RGBColor
from mpf.tests.MpfTestCase import MpfTestCase


class TestShowController(MpfTestCase):
    def getConfigFile(self):
        return 'test_shows.yaml'

    def getMachinePath(self):
        return 'tests/machine_files/show_controller/'

    def get_platform(self):
        return 'smart_virtual'

    def event_handler(self, **kwargs):
        pass

    def test_shows(self):
        # Make sure required modes have been loaded
        self.assertIn('mode1', self.machine.modes)
        self.assertIn('mode2', self.machine.modes)
        self.assertIn('mode3', self.machine.modes)

        # Make sure test shows exist and can be loaded
        self.assertIn('test_show1', self.machine.shows)
        self.assertIn('test_show2', self.machine.shows)
        self.assertIn('test_show3', self.machine.shows)

        # Make sure hardware devices have been configured for tests
        self.assertIn('led_01', self.machine.leds)
        self.assertIn('led_02', self.machine.leds)
        self.assertIn('light_01', self.machine.lights)
        self.assertIn('light_02', self.machine.lights)
        self.assertIn('gi_01', self.machine.gi)
        self.assertIn('coil_01', self.machine.coils)
        self.assertIn('flasher_01', self.machine.flashers)

        # --------------------------------------------------------
        # test_show1 - Show with LEDs, lights, and GI
        # --------------------------------------------------------

        # LEDs should start out off (current color is default RGBColor object)
        self.assertEqual(RGBColor(),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor(),
                         self.machine.leds.led_02.hw_driver.current_color)

        # Lights should start out off (brightness is 0)
        self.assertEqual(0,
                         self.machine.lights.light_01.hw_driver
                         .current_brightness)
        self.assertEqual(0,
                         self.machine.lights.light_02.hw_driver
                         .current_brightness)

        # GI should start out enabled/on (brightness is 255)
        self.assertEqual(255,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # Make sure all required shows are loaded
        start_time = time.time()
        while (not (self.machine.shows['test_show1'].loaded and
                    self.machine.shows['test_show2'].loaded and
                    self.machine.shows['test_show3'].loaded) and
                time.time() < start_time + 10):
            self.advance_time_and_run()

        self.assertTrue(self.machine.shows['test_show1'].loaded)
        self.assertEqual(self.machine.shows['test_show1'].total_steps, 6)

        # Start mode1 mode (should automatically start the test_show1 show)
        self.machine.events.post('start_mode1')
        self.advance_time_and_run(.2)
        self.assertTrue(self.machine.mode_controller.is_active('mode1'))
        self.assertTrue(self.machine.modes.mode1.active)
        self.assertIn(self.machine.modes.mode1,
                      self.machine.mode_controller.active_modes)
        self.assertTrue(self.machine.shows['test_show1'].running)

        # Check LEDs, lights, and GI after first show step
        self.assertEqual(RGBColor('006400'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('CCCCCC'),
                         self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(204,
                    self.machine.lights.light_01.hw_driver.current_brightness)
        self.assertEqual(120,
                    self.machine.lights.light_02.hw_driver.current_brightness)
        self.assertEqual(255,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # Check LEDs, lights, and GI after 2nd step
        self.advance_time_and_run(1.0)
        self.assertEqual(RGBColor('DarkGreen'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('Black'),
                         self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(204,
                    self.machine.lights.light_01.hw_driver.current_brightness)
        self.assertEqual(120,
                    self.machine.lights.light_02.hw_driver.current_brightness)
        self.assertEqual(255,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # Check LEDs, lights, and GI after 3rd step
        self.advance_time_and_run(1.0)
        self.assertEqual(RGBColor('DarkSlateGray'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('Tomato'),
                         self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(255,
                         self.machine.lights.light_01.hw_driver
                         .current_brightness)
        self.assertEqual(51,
                         self.machine.lights.light_02.hw_driver
                         .current_brightness)
        self.assertEqual(153,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # Check LEDs, lights, and GI after 4th step (includes a fade to next
        #  color)
        self.advance_time_and_run(1.0)
        self.assertNotEqual(RGBColor('MidnightBlue'),
                            self.machine.leds.led_01.hw_driver.current_color)
        self.assertNotEqual(RGBColor('DarkOrange'),
                            self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(255,
                         self.machine.lights.light_01.hw_driver
                         .current_brightness)
        self.assertEqual(51,
                         self.machine.lights.light_02.hw_driver
                         .current_brightness)
        self.assertEqual(51,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # Advance time so fade should have completed
        self.advance_time_and_run(0.1)
        self.advance_time_and_run(0.1)
        self.advance_time_and_run(0.1)
        self.advance_time_and_run(0.1)
        self.advance_time_and_run(0.1)
        self.advance_time_and_run(0.1)
        self.assertEqual(RGBColor('MidnightBlue'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('DarkOrange'),
                         self.machine.leds.led_02.hw_driver.current_color)

        # Check LEDs after 5th step (includes a fade to black/off)
        self.advance_time_and_run(0.4)
        self.assertNotEqual(RGBColor('Off'),
                            self.machine.leds.led_01.hw_driver.current_color)
        self.assertNotEqual(RGBColor('Off'),
                            self.machine.leds.led_02.hw_driver.current_color)
        self.assertNotEqual(0,
                            self.machine.lights.light_01.hw_driver.current_brightness)
        self.assertNotEqual(0,
                            self.machine.lights.light_02.hw_driver.current_brightness)

        # Advance time so fade should have completed
        self.advance_time_and_run(0.2)
        self.advance_time_and_run(0.2)
        self.advance_time_and_run(0.2)
        self.advance_time_and_run(0.2)
        self.advance_time_and_run(0.2)
        self.assertEqual(RGBColor('Off'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('Off'),
                         self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(0,
                         self.machine.lights.light_01.hw_driver
                         .current_brightness)
        self.assertEqual(0,
                         self.machine.lights.light_02.hw_driver
                         .current_brightness)
        self.assertEqual(0, self.machine.gi.gi_01.hw_driver.current_brightness)

        # Make sure show loops back to the first step
        self.advance_time_and_run(1.1)
        self.assertEqual(RGBColor('006400'),
                         self.machine.leds.led_01.hw_driver.current_color)
        self.assertEqual(RGBColor('CCCCCC'),
                         self.machine.leds.led_02.hw_driver.current_color)
        self.assertEqual(204,
                         self.machine.lights.light_01.hw_driver
                         .current_brightness)
        self.assertEqual(120,
                         self.machine.lights.light_02.hw_driver
                         .current_brightness)
        self.assertEqual(255,
                         self.machine.gi.gi_01.hw_driver.current_brightness)

        # TODO: Add tests for reset and hold

        # Stop the mode (and therefore the show)
        self.machine.events.post('stop_mode1')
        self.machine_run()
        self.assertFalse(self.machine.mode_controller.is_active('mode1'))
        self.advance_time_and_run(5)

        # --------------------------------------------------------
        # test_show2 - Show with events and triggers
        # --------------------------------------------------------

        # Setup callback for test_event event (fired in test show) and triggers
        self.event_handler = MagicMock()
        self.machine.events.add_handler('test_event', self.event_handler)
        self.machine.bcp.bcp_trigger = MagicMock()

        # Advance the clock enough to ensure the shows have time to load
        self.assertTrue(self.machine.shows['test_show2'].loaded)
        self.assertEqual(self.machine.shows['test_show2'].total_steps, 4)

        # Make sure our event callback and trigger have not been fired yet
        self.assertFalse(self.event_handler.called)
        self.assertFalse(self.machine.bcp.bcp_trigger.called)

        # Start the mode that will trigger playback of the test_show2 show
        self.machine.events.post('start_mode2')
        self.machine_run()
        self.assertTrue(self.machine.mode_controller.is_active('mode2'))
        self.assertTrue(self.machine.modes.mode2.active)
        self.assertIn(self.machine.modes.mode2,
                      self.machine.mode_controller.active_modes)
        self.assertTrue(self.machine.shows['test_show2'].running)
        self.machine_run()

        # Make sure event callback and trigger have been called
        self.assertTrue(self.event_handler.called)
        self.assertTrue(self.machine.bcp.bcp_trigger)
        self.machine.bcp.bcp_trigger.assert_called_with('play_sound',
                                                        sound="test_1",
                                                        volume=0.5, loops=-1,
                                                        priority=0)

        # Advance to next show step and check for trigger
        self.advance_time_and_run(1.0)
        self.machine.bcp.bcp_trigger.assert_called_with('play_sound',
                                                        sound="test_2",
                                                        priority=0)

        # Advance to next show step and check for trigger
        self.advance_time_and_run(1.0)
        self.machine.bcp.bcp_trigger.assert_called_with('play_sound',
                                                        sound="test_3",
                                                        volume=0.35, loops=1,
                                                        priority=0)

        # Stop the mode (and therefore the show)
        self.machine.events.post('stop_mode2')
        self.machine_run()
        self.assertFalse(self.machine.mode_controller.is_active('mode2'))
        self.advance_time_and_run(5)

        # --------------------------------------------------------
        # test_show3 - Show with coils and flashers
        # --------------------------------------------------------

        # Setup callback for test_event event (fired in test show) and triggers
        self.machine.coils['coil_01'].pulse = MagicMock()
        self.machine.flashers['flasher_01'].flash = MagicMock()

        self.assertTrue(self.machine.shows['test_show3'].loaded)
        self.assertEqual(self.machine.shows['test_show3'].total_steps, 4)

        # Make sure our device callbacks have not been fired yet
        self.assertFalse(self.machine.coils['coil_01'].pulse.called)
        self.assertFalse(self.machine.flashers['flasher_01'].flash.called)

        # Start the mode that will trigger playback of the test_show3 show
        self.machine.events.post('start_mode3')
        self.machine_run()
        self.assertTrue(self.machine.mode_controller.is_active('mode3'))
        self.assertTrue(self.machine.modes.mode3.active)
        self.assertIn(self.machine.modes.mode3,
                      self.machine.mode_controller.active_modes)
        self.assertTrue(self.machine.shows['test_show3'].running)
        self.machine_run()

        # Make sure flasher device callback has been called (in first step
        # of show)
        self.assertTrue(self.machine.flashers['flasher_01'].flash.called)

        # Advance to next show step and check for coil firing
        self.advance_time_and_run(1.0)
        self.machine.coils['coil_01'].pulse.assert_called_with(action='pulse',
                                                               power=1.0)

        # Advance to next show step and check for coil firing
        self.advance_time_and_run(1.0)
        self.machine.coils['coil_01'].pulse.assert_called_with(
            action='pulse', power=0.45, ms=0, priority=0)

        # TODO: Test device tags
        # TODO: Add test for multiple shows running at once with different
        # priorities
        # TODO: Test show playback rate

    def test_token_processing(self):

        data = '''
                - time: 0
                  (leds): red
                - time: +1
                  (leds): off
                - time: +1
                  (leds):
                    some: setting
                    (other): setting
                    this: (foo)
            '''

        data = yaml.load(data)
        show = Show(self.machine, name='test', data=data)

        self.assertIn('leds', show.token_keys)
        self.assertIn('other', show.token_keys)
        self.assertIn('foo', show.token_values)

        self.assertIn([0], show.token_keys['leds'])
        self.assertIn([1], show.token_keys['leds'])
        self.assertIn([2], show.token_keys['leds'])

        self.assertIn([2, '(leds)', 'this'], show.token_values['foo'])

        test_show = show._replace_tokens(foo='hello')
        self.assertEqual(test_show[2]['(leds)']['this'], 'hello')

        test_show = show._replace_tokens(leds='hello')
        self.assertIn('hello', test_show[0])
        self.assertIn('hello', test_show[1])
        self.assertIn('hello', test_show[2])

        # test multiples at the same time
        test_show = show._replace_tokens(leds='hello', other='other',
                                             foo='hello')
        self.assertEqual('hello', test_show[2]['hello']['this'])
        self.assertIn('hello', test_show[0])
        self.assertIn('hello', test_show[1])
        self.assertIn('hello', test_show[2])

    def test_tokens_in_shows(self):
        self.assertIn('leds_basic', self.machine.shows)
        self.assertIn('leds_basic_fade', self.machine.shows)
        self.assertIn('leds_color_token', self.machine.shows)
        self.assertIn('leds_extended', self.machine.shows)
        self.assertIn('lights_basic', self.machine.shows)
        self.assertIn('multiple_tokens', self.machine.shows)

        self.machine.events.post('play_leds_basic_single')
        self.advance_time_and_run(.5)

    def test_get_show_copy(self):
        copied_show = self.machine.shows['test_show1'].get_show_steps()
        self.assertEqual(6, len(copied_show))
        self.assertIs(type(copied_show), list)
        self.assertEqual(copied_show[0]['time'], 0)
        self.assertEqual(copied_show[1]['time'], 1.0)
        self.assertEqual(copied_show[2]['time'], 1.0)
        self.assertEqual(copied_show[4]['time'], 1.0)
        self.assertEqual(copied_show[5]['time'], 2.0)
        self.assertIn(self.machine.leds.led_01, copied_show[0]['leds'])
        self.assertIn(self.machine.leds.led_02, copied_show[0]['leds'])
        self.assertEqual(copied_show[0]['leds'][self.machine.leds.led_01],
                         dict(color='006400', fade_ms=0))
        self.assertEqual(copied_show[0]['leds'][self.machine.leds.led_02],
                         dict(color='cccccc', fade_ms=0))
        self.assertEqual(copied_show[3]['leds'][self.machine.leds.led_01],
                         dict(color='midnightblue', fade_ms=500))