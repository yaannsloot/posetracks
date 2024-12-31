typedef struct { int   x;
    struct {
        float y;
        char  z ;
    } NestedStruct;
} ComplexStruct;

struct ComplexStruct2 {
    int *   ptr  DNA_DEPRECATED;
    char str[100];
    float x, y, z;
};

enum week{Mon, Tue, Wed};
enum week day;