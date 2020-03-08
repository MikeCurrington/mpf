"""Loads MPF configs."""

import logging
import os
import pickle
import sys

from pathlib import PurePath

from mpf.core.config_processor import ConfigProcessor


class MpfConfig:

    """Contains a MPF config."""

    __slots__ = ["_config_spec", "_machine_config", "_mode_config", "_show_config", "_machine_path", "_mpf_path"]

    # pylint: disable-msg=too-many-arguments
    def __init__(self, config_spec, machine_config, modes, shows, machine_path, mpf_path):
        """Initialize config."""
        self._config_spec = config_spec
        self._machine_config = machine_config
        self._mode_config = modes
        self._show_config = shows
        self._machine_path = machine_path
        self._mpf_path = mpf_path

    def get_mpf_path(self):
        """Return mpf path."""
        return self._mpf_path

    def get_machine_path(self):
        """Return machine path."""
        return self._machine_path

    def get_config_spec(self):
        """Return config spec."""
        return self._config_spec

    def get_machine_config(self):
        """Return machine wide config."""
        return self._machine_config

    def get_mode_config(self, mode_name):
        """Return config for a mode."""
        try:
            return self._mode_config[mode_name]
        except KeyError:
            raise AssertionError("No config found for mode '{mode_name}'. MPF expects the config at "
                                 "'modes/{mode_name}/config/{mode_name}.yaml' inside your machine "
                                 "folder.".format(mode_name=mode_name))

    def get_modes(self):
        """Return a list of mode names."""
        return self._mode_config.keys()

    def get_show_config(self, show_name):
        """Return a show."""
        try:
            return self._show_config[show_name]
        except KeyError:
            raise AssertionError("No config found for show '{}'.".format(show_name))

    def get_shows(self):
        """Return a list of all shows names."""
        return self._show_config.keys()


class ConfigLoader:

    """Generic loader for MPF and MC configs."""

    __slots__ = []

    def load_mpf_config(self) -> MpfConfig:
        """Load and return a MPF config."""

    def load_mc_config(self) -> MpfConfig:
        """Load and return a MC config."""


class YamlMultifileConfigLoader(ConfigLoader):

    """Loads MPF configs from machine folder with config and modes."""

    __slots__ = ["configfile", "machine_path", "mpf_path", "config_processor", "log"]

    # pylint: disable-msg=too-many-arguments
    def __init__(self, machine_path, mpf_path, configfile, load_cache, store_cache):
        """Initialize yaml multifile config loader."""
        self.configfile = configfile
        self.machine_path = machine_path
        self.mpf_path = mpf_path
        self.config_processor = ConfigProcessor(load_cache, store_cache)
        self.log = logging.getLogger("YamlMultifileConfigLoader")

    def load_mpf_config(self) -> MpfConfig:
        """Load and return a MPF config."""
        config_spec = self._load_config_spec()
        machine_config = self._load_mpf_machine_config(config_spec)
        config_spec = self._load_additional_config_spec(config_spec, machine_config)
        mode_config = self._load_modes(config_spec, machine_config)
        show_config = self._load_shows(config_spec, machine_config, mode_config)
        return MpfConfig(config_spec, machine_config, mode_config, show_config, self.machine_path, self.mpf_path)

    def _load_config_spec(self):
        return self.config_processor.load_config_spec()

    def _load_mpf_machine_config(self, config_spec):
        config_files = [os.path.join(self.mpf_path, "mpfconfig.yaml")]

        for num, config_file in enumerate(self.configfile):
            config_files.append(os.path.join(self.machine_path, "config", config_file))

            self.log.info("Machine config file #%s: %s", num + 1, config_file)

        return self.config_processor.load_config_files_with_cache(config_files, "machine", config_spec=config_spec)

    def _load_additional_config_spec(self, config_spec, machine_config):
        """Load additional config specs from devices."""
        sys.path.insert(0, self.machine_path)
        config_spec = self.config_processor.load_device_config_specs(config_spec, machine_config)
        sys.path.remove(self.machine_path)
        return config_spec

    def _load_modes(self, config_spec, machine_config):
        mode_path = machine_config['mpf']['paths']['modes']
        mode_config = {}
        for mode in machine_config.get("modes", {}):
            mpf_config_path = os.path.join(self.mpf_path, "modes", mode, 'config', mode + '.yaml')
            machine_config_path = os.path.join(self.machine_path, mode_path, mode, 'config', mode + '.yaml')
            mode_config_files = []
            if os.path.isfile(mpf_config_path):
                self.log.debug("Loading mode %s from %s", mode, mpf_config_path)
                mode_config_files.append(mpf_config_path)
            if os.path.isfile(machine_config_path):
                self.log.debug("Loading mode %s from %s", mode, mpf_config_path)
                mode_config_files.append(machine_config_path)

            if not mode_config_files:
                raise AssertionError("No config found for mode '{mode_name}'. MPF expects the config at "
                                     "'modes/{mode_name}/config/{mode_name}.yaml' inside your machine "
                                     "folder.".format(mode_name=mode))

            config = self.config_processor.load_config_files_with_cache(mode_config_files, "mode",
                                                                        config_spec=config_spec)

            # TODO: load mode config spec here

            if "mode" not in config:
                config["mode"] = dict()

            mode_config[mode] = config
        return mode_config

    def _load_shows_in_folder(self, folder, show_configs, config_spec):
        if not os.path.isdir(folder):
            return show_configs
        # ignore temporary files
        ignore_prefixes = (".", "~")
        # do not get fooled by windows or mac garbage
        ignore_files = ("desktop.ini", "Thumbs.db")

        for this_path, _, files in os.walk(folder, followlinks=True):
            relative_path = PurePath(this_path).relative_to(folder)
            for show_file_name in [f for f in files if f.endswith(".yaml") and not f.startswith(ignore_prefixes) and
                                   f != ignore_files]:
                show_name = show_file_name[:-5]
                if show_name in show_configs:
                    raise AssertionError("Duplicate show {}".format(show_name))
                show_config = self.config_processor.load_config_files_with_cache(
                    [os.path.join(folder, relative_path, show_file_name)], "show", config_spec=config_spec)
                show_configs[show_name] = show_config
        return show_configs

    def _load_shows(self, config_spec, machine_config, mode_config):
        show_configs = {}
        for show_name, show_config in machine_config.get("shows", {}).items():
            show_configs[show_name] = show_config

        show_configs = self._load_shows_in_folder(os.path.join(self.machine_path, "shows"), show_configs, config_spec)

        mode_path = machine_config['mpf']['paths']['modes']
        for mode_name, config in mode_config.items():
            for show_name, show_config in config.get("shows", {}).items():
                if show_name in show_configs:
                    raise AssertionError("Duplicate show {}".format(show_name))
                show_configs[show_name] = show_config

            show_configs = self._load_shows_in_folder(os.path.join(self.mpf_path, "modes", mode_name, 'shows'),
                                                      show_configs, config_spec)
            show_configs = self._load_shows_in_folder(os.path.join(self.machine_path, mode_path, mode_name, 'shows'),
                                                      show_configs, config_spec)

        return show_configs


class ProductionConfigLoader(ConfigLoader):

    """Loads a single pickled production config."""

    __slots__ = []

    def load_mpf_config(self) -> MpfConfig:
        """Load and return a MPF config."""
        return pickle.load("mpf_config_bundle.bin")
