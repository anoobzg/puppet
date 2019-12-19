#include <iostream>
using namespace std;

int f(int n)
{
	if (n == 1) return 1;
	if (n == 2) return 2;
	return f(n - 1) + f(n - 2);
}

int main()
{
	int n;
	while (cin >> n)
	{
		if (n <= 0)
		{
			cout << "n must be positive." << endl;
			continue;
		}

		int r = f(n);
		cout << r << endl;
	}
}