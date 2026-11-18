"""Language-neutral operator messages shared by the GUI and SDK worker.

Keep English source templates and named values separate. Calibration files and
robot commands never depend on the current display language.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Message:
    source: str
    values: dict = field(default_factory=dict)

    def english(self):
        return self.source.format(**{
            key: value.english() if isinstance(value, Message) else value
            for key, value in self.values.items()
        })

    def payload(self):
        return {'source': self.source, 'values': {
            key: {'message': value.payload()} if isinstance(value, Message) else value
            for key, value in self.values.items()
        }}

    @classmethod
    def from_payload(cls, data):
        return cls(data['source'], {
            key: cls.from_payload(value['message']) if isinstance(value, dict) and 'message' in value else value
            for key, value in data.get('values', {}).items()
        })


class OperatorValueError(ValueError):
    def __init__(self, source, **values):
        self.message = Message(source, values)
        super().__init__(self.message.english())


class OperatorError(RuntimeError):
    def __init__(self, source, **values):
        self.message = Message(source, values)
        super().__init__(self.message.english())


class MoveRejected(OperatorError):
    """A validated refusal before any motion command was sent to the controller."""


def error_message(error):
    if isinstance(error, (OperatorError, OperatorValueError)):
        return error.message
    # Preserve original vendor/OS details for diagnosis without pretending to
    # translate arbitrary third-party error strings.
    return Message('Operation failed: {detail}', {'detail': str(error)})
