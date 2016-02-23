template <uint16_t time>
class interval_t {
	uint32_t next_run = 0;

public:
	template <class T>
	void run(T func) {
		uint32_t now = millis();
		if (next_run < now) {
			func();
			next_run = now + time;
		}
	}
};

template <uint16_t time, class T>
void interval(T func) {
	static interval_t<time> instance;
	instance.run(func);
}

