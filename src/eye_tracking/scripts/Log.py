class Log:
    @classmethod
    def d(cls, tag, content):
        print(f"[{tag}]: {content}")