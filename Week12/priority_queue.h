typedef struct{
    int row;
    int col;
    float weight;
} cell;

cell frontier[MAX_CELL];
int frontierCount;

cell popLeastWeight(){
    cell tmp;

    if(frontierCount == 0)
        return tmp;

    float min = INFINITY;
    int index = 0;
    for(int i = 0; i < frontierCount; i++){
        if(frontier[i].weight < min){
            max = frontier[i].weight;
            index = i;
        }
    }
    tmp.row = frontier[index].row;
    tmp.col = frontier[index].col;
    tmp.weight = frontier[index].weight;
    for(int i = index + 1; i < frontierCount; i++){
        frontier[i - 1] = frontier[i];
    }
    return tmp;
}

cell 