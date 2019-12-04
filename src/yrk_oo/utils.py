class Pointer(int):
    def __getitem__(self, key):
        return Pointer(self + key)
