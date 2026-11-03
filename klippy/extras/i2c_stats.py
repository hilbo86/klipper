# Host-side diagnostics for I2C transfers
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import logging

WINDOW_SECONDS = 3600
LOG_INTERVAL = 60.


class I2CClientStats:
    def __init__(self, client, name):
        self.client = client
        self.name = name
        # Keep at most one bucket per second, even during an error storm.
        # The rolling hour has one-second resolution; a bucket expires at
        # the start of its expiry second (at most one second early).
        self.buckets = collections.deque()
        self.window_counts = collections.Counter()
        self.errors_total = self.skipped_total = 0
        self.last_error = self.last_error_time = None
        self.next_log_time = 0.

    def _discard_oldest(self):
        second, counts = self.buckets.popleft()
        for status, count in counts.items():
            remaining = self.window_counts[status] - count
            if remaining:
                self.window_counts[status] = remaining
            else:
                del self.window_counts[status]

    def _expire(self, eventtime):
        cutoff = int(eventtime) - WINDOW_SECONDS
        while self.buckets and self.buckets[0][0] <= cutoff:
            self._discard_oldest()

    def note_error(self, status, eventtime):
        self._expire(eventtime)
        second = int(eventtime)
        if not self.buckets or self.buckets[-1][0] != second:
            if len(self.buckets) == WINDOW_SECONDS:
                self._discard_oldest()
            self.buckets.append((second, collections.Counter()))
        self.buckets[-1][1][status] += 1
        self.window_counts[status] += 1
        self.errors_total += 1
        self.last_error = status
        self.last_error_time = eventtime
        if eventtime >= self.next_log_time:
            self.next_log_time = eventtime + LOG_INTERVAL
            message = (
                "I2C error: client=%s mcu=%s bus=%s oid=%d address=0x%02x"
                " status=%s errors_total=%d (messages limited to once per 60s)"
                % (self.name, self.client.get_mcu().get_name(), self.client.bus,
                   self.client.get_oid(), self.client.get_i2c_address(), status,
                   self.errors_total))
            logging.warning('%s', message)
            return message

    def get_status(self, eventtime):
        # Polls only visit buckets that have expired since the last update.
        self._expire(eventtime)
        return {
            'name': self.name,
            'mcu': self.client.get_mcu().get_name(),
            'oid': self.client.get_oid(),
            'bus': self.client.bus,
            'address': self.client.get_i2c_address(),
            'errors_last_hour': sum(self.window_counts.values()),
            'errors_total': self.errors_total,
            'by_status_last_hour': dict(self.window_counts),
            'last_error': self.last_error,
            'last_error_time': self.last_error_time,
            'skipped_total': self.skipped_total,
        }


class I2CStats:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.clients = {}
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'IIC_ERRORS', self.cmd_IIC_ERRORS, desc=self.cmd_IIC_ERRORS_help)

    def _client_key(self, client):
        return '%s:%d' % (client.get_mcu().get_name(), client.get_oid())

    def register_client(self, client, name):
        key = self._client_key(client)
        if key not in self.clients:
            self.clients[key] = I2CClientStats(client, name)

    def note_error(self, client, status):
        if status == 'SUCCESS':
            return
        message = self.clients[self._client_key(client)].note_error(
            status, self.reactor.monotonic())
        if message is not None:
            # Called on the reactor thread for both synchronous transfers and
            # marshalled serial replies. Also reaches Mainsail/Fluidd consoles.
            self.gcode.respond_info(message, log=False)

    def note_skip(self, client):
        # A request suppressed during cooldown is not another bus error.
        self.clients[self._client_key(client)].skipped_total += 1

    def get_status(self, eventtime):
        clients = {key: stats.get_status(eventtime)
                   for key, stats in self.clients.items()}
        return {
            'window_seconds': WINDOW_SECONDS,
            'resolution_seconds': 1,
            'errors_last_hour': sum(s['errors_last_hour']
                                    for s in clients.values()),
            'errors_total': sum(s['errors_total'] for s in clients.values()),
            'clients': clients,
        }

    cmd_IIC_ERRORS_help = 'Report I2C errors in the last hour by client'

    def cmd_IIC_ERRORS(self, gcmd):
        status = self.get_status(self.reactor.monotonic())
        lines = [
            'I2C errors in the last hour: %d; total since Klippy start: %d'
            ' (1s resolution)' % (status['errors_last_hour'],
                                  status['errors_total'])]
        for key, client in sorted(status['clients'].items()):
            types = ', '.join('%s=%d' % item for item in
                              sorted(client['by_status_last_hour'].items()))
            lines.append(
                '"%s": mcu=%s bus=%s oid=%d address=0x%02x'
                ' last_hour=%d total=%d skipped=%d types=[%s] last=%s' % (
                    client['name'], client['mcu'], client['bus'],
                    client['oid'], client['address'],
                    client['errors_last_hour'], client['errors_total'],
                    client['skipped_total'], types, client['last_error']))
        gcmd.respond_info('\n'.join(lines))


def load_config(config):
    return I2CStats(config)
