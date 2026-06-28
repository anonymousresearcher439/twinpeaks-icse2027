class MessageHandler:
    def __init__(self):
        self.passive_handlers = {}
        self.active_handlers = {}

    def add_handler(self, message_type, callback, is_active=True):
        if is_active:
            self._add_handler(self.active_handlers, message_type, callback)
        else:
            self._add_handler(self.passive_handlers, message_type, callback)

    def notify(self, message):
        message_type = message["type"]

        if message_type in self.passive_handlers:
            passive_callbacks = self.passive_handlers[message_type]
            for func in passive_callbacks:
                func(message)
        if message_type in self.active_handlers:
            active_callbacks = self.active_handlers[message_type]
            for func in active_callbacks:
                result = func(message)
                if result is not None:
                    return result
        return None

    def _add_handler(self, lookup_table, message_type, callback):
        if message_type not in lookup_table:
            lookup_table[message_type] = []
        if callback not in lookup_table[message_type]:
            lookup_table[message_type].append(callback)