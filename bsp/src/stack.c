#include "stack.h" 

void Stack_Init(Stack *S)
{
    S->size = 0;
}

double Stack_Top(Stack *S)
{
    if (S->size == 0) {
        fprintf(stderr, "Error: stack empty\n");
        return -1;
    } 

    return S->data[S->size-1];
}

void Stack_Push(Stack *S, double d)
{
    if (S->size < STACK_MAX)
        S->data[S->size++] = d;
    else
        fprintf(stderr, "Error: stack full\n");
}

void Stack_Pop(Stack *S)
{
    if (S->size == 0)
        fprintf(stderr, "Error: stack empty\n");
    else
        S->size--;
}

double Stack_totalVal(Stack *S){
    double sum = 0;
    for(int i=0; i < S->size; i++){
        sum += S->data[i];
    }
    return sum;
}