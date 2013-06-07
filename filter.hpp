#ifndef KVETINAC_FILTER_HPP
#define KVETINAC_FILTER_HPP

#ifndef KVETINAC_FILTER_WEIGHTED_MOVING_AVERAGE_MAX_PERIOD
#define KVETINAC_FILTER_WEIGHTED_MOVING_AVERAGE_MAX_PERIOD 32
#endif

template <typename Value_type>
struct filter_base
{
	typedef Value_type value_type;
	
	virtual void input(value_type value) = 0;
	virtual value_type output() = 0;
};

template <typename Value_type>
struct no_filter
	:filter_base<Value_type>
{
	typedef Value_type value_type;
	
	no_filter()
		:m_value(0)
	{}
	void input(value_type value) { m_value = value; }
	value_type output() { return m_value; }
protected:
	volatile value_type m_value;
};

template <typename Value_type ,typename Internal_type = Value_type, uint16_t Max_capacity = KVETINAC_FILTER_WEIGHTED_MOVING_AVERAGE_MAX_PERIOD, uint8_t Precision = 6>
class weighted_moving_average
	:public filter_base<Value_type>
{
public:
	typedef Value_type value_type;
	typedef Internal_type internal_type;
	typedef uint8_t index_type;
	typedef uint8_t weight_type;
	
	weighted_moving_average(index_type Period = 16, weight_type Input_weight = 1, 
							weight_type Output_weight = 16, value_type Noise_level = -1, 
							value_type initial_value = 0)
		:m_value(initial_value), m_index(0), m_period(Period), m_input_weight(Input_weight),
		 m_output_weight(Output_weight), m_noise_level(Noise_level), m_sum(Period * initial_value)
	{
		for(index_type i = 0; i != c_max_period; ++i)
			m_buffer[i] = initial_value;
	}

	void input(value_type value)
	{
		internal_type Value = value;
		Value <<= c_precision;
		if(abs(value - output()) > m_noise_level)
		{
			for(index_type i = 0; i != m_period; ++i)
				m_buffer[i] = value;
			m_sum = m_period * Value;
			m_value = Value;
		}
		else
		{
			m_sum -= (m_buffer[m_index]<<c_precision);
			if(++m_index >= m_period)
				m_index = 0;
			m_buffer[m_index] = value;
			m_sum += Value;
			internal_type sum = ((m_sum / m_period) * m_input_weight + m_value * m_output_weight);
			m_value = (sum / (m_input_weight + m_output_weight));
		}
	}
	
	value_type output() { return (m_value>>c_precision); }
		
	void input_weight(weight_type v) { m_input_weight = v; }
	weight_type input_weight() { return m_input_weight; }
		
	void output_weight(weight_type v) { m_output_weight = v; }
	weight_type output_weight() { return m_output_weight; }
		
	void period(index_type v) { m_index = v; }
	index_type period() { return m_index; }
		
	void noise_level(value_type v) { m_noise_level = v; }
	value_type noise_level() { return m_noise_level; }
		
	static const index_type c_max_period = Max_capacity;
	static const index_type c_precision = Precision;
	
protected:
	volatile internal_type m_value;
	index_type m_index;
	volatile index_type m_period;
	volatile weight_type m_input_weight;
	volatile weight_type m_output_weight;
	volatile value_type m_noise_level;
	internal_type m_sum;
	value_type m_buffer[c_max_period];
};

#endif