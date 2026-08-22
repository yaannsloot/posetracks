#define suzew 100

typedef void (*w);

// STRUCT DECL for ComplexStruct
typedef struct {
  w func;
  int (*x)[4]; // FIELD DECL
  struct { // STRUCT DECL for unnamed. Includes fields
    float y;
    char z;
  } NestedStruct, (*NestedStruct2)[4]; 
  // Following, there is a FIELD_DECL for NestedStruct, 
  // and the STRUCT_DECL repeats with a FIELD DECL for NestedStruct2
} (*ComplexStruct)[5];

struct ComplexStruct2 {
  int *ptr DNA_DEPRECATED;
  char str[10];
  float x, y, z;
};

typedef struct {
  int x;
  struct AStruct {
    float y;
    char z;
  } NestedStruct, NestedStruct2;
  struct AStruct (*the4)[4];
} ComplexStruct3;

typedef struct NoStruct {
  int b;
  ComplexStruct3 other;
  ComplexStruct other2;
} TheStruct;

typedef struct {
  int x;
} TheStruct2;

typedef struct {
  int a;
} BStruct;

enum week { Mon, Tue, Wed };
enum week day;

// Reducing pointer declarations to generics (void)
//
// In the AST (abstract syntax tree) implementation there are 3 modifiers for declarations, ptr, array, and parenthesis.
// These are stored in reverse order of the right left rule. (See https://cseweb.ucsd.edu/~gbournou/CSE131/rt_lt.rule.html)
// *(*c[4])[4] would produce ptr, array(4), parenthesis, ptr, array(4)
// This is due to the order the modifiers are applied to get the original text.
// ptr = *c, array(4) = *c[4], parenthesis = (*c[4]), ptr = *(*c[4]), array(4) = *(*c[4])[4]
//
// Function returning is omitted because it is not expected to occur within the targeted codebase
//
// The goal is to determine when a typed declaration can be reduced to void
// and the minimum declaration that preserves memory layout.
//
// Why? Because only a portion of the codebase is being analysed and may not include all dependencies.
//
// Examples: 
struct TheStruct decl; // cannot be changed to void because it is an instantiation of TheStruct 
// The memory layout for this var would be the layout of TheStruct
void *(*a[4]); // Is an array of pointers to pointers
// The base type is an array of pointers and can be reduced to void *a[4];
void *b[4]; // Is an array of pointers. Cannot be changed further.
void *(*c[4])[4]; // Is an array of pointers to arrays of pointers. 
// The base type is an array of pointers and can be reduced to void *c[4];
void *(*d[4])[4][4]; // Is an array of pointers to an array of arrays of pointers. 
// The base type is an array of pointers and can be reduced to void *d[4];
void *(*e[4][4])[4][4]; // Is an array of arrays of pointers to an array of arrays of pointers. 
// The base type is an array of arrays of pointers and can be reduced to void *e[4][4];
void *((*f[4])[4])[4][4]; // Is an array of pointers to an array of arrays of arrays of pointers. 
// The base type is an array of pointers and can be reduced to void *f[4];
void *((*g)[4])[4][4]; // Is a pointer to an array of arrays of arrays of pointers.
// The base type is a pointer and can be reduced to void *g;
// void is typed above, but in practice could be any tagged structure

// Solution
// Evaluate modifiers in reverse order
// *(*c[4])[4] would produce ptr, array(4), parenthesis, ptr, array(4)
// So reversed would be array(4), ptr, parenthesis, array(4), ptr
// Parenthesis modifiers can be ignored as they are only used to preserve the definition in C.
// If the order starts with a pointer it is a pointer
// If the order starts with an array continue until first pointer
// If no pointer is found it cannot be voided