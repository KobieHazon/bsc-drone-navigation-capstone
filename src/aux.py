import os
import numbers
import math

# ~~~~~~~~~~~~~~~~~~~~~~~~~ WIFI_FINDER CLASS ~~~~~~~~~~~~~~~~~~~~~~~~~~~


class Wifi_Finder:
    def __init__(self, *args, **kwargs):
        self.server_name = kwargs['server_name']
        self.password = kwargs['password']
        self.interface_name = kwargs['interface']
        self.main_dict = {}

    def run(self):
        command = """sudo iwlist {} scan | grep -ioE 'ssid:"(.*{}.*)'"""
        result = os.popen(command.format(self.interface_name, self.server_name))
        result = list(result)

        if "Device or resource busy" in result:
            return None
        else:
            ssid_list = [item.lstrip('SSID:').strip('"\n') for item in result]
            # print("Successfully get ssids {}".format(str(ssid_list)))

        for name in ssid_list:
            try:
                result = self.connection(name)
            except Exception as exp:
                print("Couldn't connect to name : {}. {}".format(name, exp))
            else:
                if result:
                    print("Successfully connected to {}".format(name))
                    return True

    def connection(self, name):
        try:
            os.system("nmcli d wifi connect {} password {} iface {}".format(name,
                                                                            self.password,
                                                                            self.interface_name))
        except Exception:
            raise
        else:
            return True


# ~~~~~~~~~~~~~~~~~ RANGEDICT CLASS AND METHODS ~~~~~~~~~~~~~~~~~~~

def in_range(some_range, item):
    if (item <= some_range[1]) and (item >= some_range[0]):
        return True
    return False


def is_range(some_range):
    if isinstance(some_range, tuple) and len(some_range) == 2 and isinstance(some_range[0], numbers.Number) and \
            isinstance(some_range[1], numbers.Number) and some_range[0] < some_range[1]:
        return True
    return False


class RangeDict(dict):
    def __getitem__(self, item):
        if isinstance(item, numbers.Number):
            for key in self:
                if in_range(key, item):
                    return super(RangeDict, self).__getitem__(key)
        else:
            raise KeyError(
                "RangeDict can only be queried by a number. KeyError " + str(item))  # or some error I can create

    def not_intersect(self, new_range):
        for key in self:
            if not ((new_range[0] <= key[0] and new_range[1] <= key[0]) or (
                    new_range[1] >= key[1] and new_range[0] >= key[1])):
                return False
        return True

    def __setitem__(self, key, value):
        if is_range(key) and self.not_intersect(key):
            return super(RangeDict, self).__setitem__(key, value)
        raise KeyError("RangeDict can only be set by a range tuple that doesn't intersect other ranges.")
