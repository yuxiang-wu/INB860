typedef struct{
    int row;
    int col;
    float weight;
} cell;

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
	
	// delete the cell from frontier array
    for(int i = index + 1; i < frontierCount; i++){
        frontier[i - 1] = frontier[i];
    }
	frontierCount--;
	
	// move label of the cell from frontier to explored
	front[tmp.row][tmp.col] = 0;
	explored[tmp.row][tmp.col] = 1;
    return tmp;
}

cell updateFrontier(cell c){
	bool inFrontier = false;
	for(int i = 0; i < frontierCount; i++){
		if(frontier[i].row == c.row && frontier.col == c.col && frontier.weight > c.weight){
			frontier[i].weight = c.weight;
			inFrontier = true;
			break;
		}
	}
	
	// add to the frontier
	if(!inFrontier){
		frontier[frontierCount] = c;
		front[c.row][c.col] = 1;
		frontierCount++;
	}
}