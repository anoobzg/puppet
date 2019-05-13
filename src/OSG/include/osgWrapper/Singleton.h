#ifndef VTD_SINGLETON_H
#define VTD_SINGLETON_H

namespace OSGWrapper
{
template<class T>
class Singleton {
protected:
    static void init() {
        m_singleton_class = new T;
    }

	static bool isInited() {
		return (m_singleton_class != 0);
	}
public:
    static void Delete() {
        delete m_singleton_class;
        m_singleton_class = 0;
    }

    static T* GetPtr() {
		if (!isInited())
			init();
        return m_singleton_class;
    }

    static T& GetRef() {
		if (!isInited())
			init();

        return *m_singleton_class;
    }

private:
    static T* m_singleton_class;
};

/// init static pointers with 0
template<class T>
T* Singleton<T>::m_singleton_class = 0;

}
#endif // VTD_SINGLETON_H
