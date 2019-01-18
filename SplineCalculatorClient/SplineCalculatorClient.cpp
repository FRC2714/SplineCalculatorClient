#include "easywsclient.hpp"
#include <iostream>
#include <assert.h>

using namespace std;
using namespace easywsclient;

int main()
{
	WebSocket * client = WebSocket::from_url("ws://localhost:554");
	assert(client);
	string s;
	cin >> s;
	delete client;
    cout << "Hello World!\n";
}

