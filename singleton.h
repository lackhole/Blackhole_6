#pragma once

template <typename _Ty>
class _Singleton {
protected:
	_Singleton() {}

public:
	static _Ty& getInstance() {
		static _Ty instance;

		return instance;
	}

private:

};
