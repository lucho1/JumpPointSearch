// ----------------------------------------------------
// Prority FIFO implementation with single linked list  -------
// ----------------------------------------------------

#ifndef __p2PQueue_H__
#define __p2PQueue_H__

#include "p2Defs.h"

/**
* Contains items from double linked list
*/
template<class tdata>
struct p2PQueue_item
{
	tdata					data;
	int						priority;
	p2PQueue_item<tdata>*   next = nullptr;

	p2PQueue_item(const tdata& _data, int priority) : data(_data), priority(priority)
	{}
};

/**
* Manages a double linked list
*/
template<class tdata>
class p2PQueue
{

public:

	p2PQueue_item<tdata>*   start;

public:

	/**
	* Constructor
	*/
	inline p2PQueue() : start(NULL)
	{}

	/**
	* Destructor
	*/
	~p2PQueue()
	{
		Clear();
	}

	/**
	* Get Size
	*/
	unsigned int Count() const
	{
		unsigned int result = 0;
		p2PQueue_item<tdata>* tmp;

		for (tmp = start; tmp != NULL; tmp = tmp->next)
			++result;

		return result;
	}

	/**
	* Get last item
	*/
	p2PQueue_item<tdata>* GetLast()
	{
		p2PQueue_item<tdata>* tmp = start;

		while (tmp != NULL && tmp->next != NULL)
			tmp = tmp->next;

		return tmp;
	}

	/**
	* push new item
	*/
	void Push(const tdata& item, int priority)
	{
		p2PQueue_item<tdata>*   p_data_item;
		p_data_item = new p2PQueue_item < tdata >(item, priority);

		p2PQueue_item<tdata>* last = GetLast();

		if (last == NULL)
		{
			start = p_data_item;
		}
		else
		{
			p2PQueue_item<tdata>* prev = start;
			p2PQueue_item<tdata>* tmp = start;

			while (tmp && tmp->priority <= priority)
			{
				prev = tmp;
				tmp = tmp->next;
			}

			p_data_item->next = tmp;
			if (tmp == start)
				start = p_data_item;
			else
				prev->next = p_data_item;
		}
	}

	/**
	* Deletes an item from the list
	*/
	bool Pop(tdata& item)
	{
		bool result = false;

		if (start != nullptr)
		{
			p2PQueue_item<tdata>* new_start = start->next;
			item = start->data;
			RELEASE(start);
			start = new_start;
			result = true;
		}

		return result;
	}

	/**
	* Reads one of the values
	*/
	const tdata* Peek(unsigned int index) const
	{
		unsigned int i = 0;
		p2PQueue_item<tdata>* tmp;

		for (tmp = start; tmp != NULL && i < index; tmp = tmp->next)
			++i;

		if (tmp != NULL)
			return &(tmp->data);

		return NULL;
	}

	/**
	* Destroy and free all mem
	*/
	void Clear()
	{
		p2PQueue_item<tdata>*   p_data;
		p2PQueue_item<tdata>*   p_next;
		p_data = start;

		while (p_data != NULL)
		{
			p_next = p_data->next;
			RELEASE(p_data);
			p_data = p_next;
		}

		start = NULL;
	}

};
#endif /*__p2PQueue_H__*/	

