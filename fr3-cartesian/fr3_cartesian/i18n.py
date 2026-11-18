"""Central translation catalog; presentation changes never touch robot state."""
import json
from importlib.resources import files
from .messages import Message

LANGUAGES = {'en': 'English', 'zh_CN': '简体中文'}
ZH_CN = json.loads(files('fr3_cartesian').joinpath('locales/zh_CN.json').read_text(encoding='utf-8'))


def translate(language, text, **values):
    message = text if isinstance(text, Message) else Message(text, values)
    template = ZH_CN.get(message.source, message.source) if language == 'zh_CN' else message.source
    return template.format(**{
        key: translate(language, value) if isinstance(value, Message) else value
        for key, value in message.values.items()
    })
