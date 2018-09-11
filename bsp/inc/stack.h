#ifndef __STACK_H
#define __STACK_H

#define STACK_MAX 8

struct Stack {
    double  data[STACK_MAX];
    int     size;
};
typedef struct Stack Stack;

void Stack_Init(Stack *S);

double Stack_Top(Stack *S);

void Stack_Push(Stack *S, double d);

void Stack_Pop(Stack *S);

double Stack_Avg(Stack *S);

#endif
