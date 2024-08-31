#include <iostream>
#include<vector>
using namespace std;
void bubblesort(vector<int> &list) {
	int len = list.size();
	for (int i = 0; i < len-1; i++) {
		bool change = false;
		for (int j = 0; j < len - 1 - i; j++) {
			if (list[j] > list[j + 1]) {
				int tmp = list[j];
				list[j] = list[j + 1];
				list[j + 1] = tmp;
				change = true;
			}
		}
		if (change == false) {
			break;
		}
	}

}
int main() {
	vector<int> list = { 10,5,7,9,3 };
	bubblesort(list);
	int l = list.size();
	for (int i = 0; i < l; i++) {
		cout << list[i] << " ";
	}
	return 0;
}
