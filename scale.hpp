#ifndef KVETINAC_SCALE_HPP
#define KVETINAC_SCALE_HPP

template<typename Value_type, typename Internal_type = Value_type>
class scale_t
{
	typedef Internal_type internal_type;
public:
	typedef Value_type value_type;
	
	scale_t(value_type Mult, value_type Div, value_type Offset)
		:m_mult(Mult), m_div(Div), m_offset(Offset)
	{}
		
	void mult(const value_type& v) { m_mult = v; }
	value_type mult() const {return m_mult; }
		
	void div(const value_type& v) { m_div = v; }
	value_type div() const {return m_div; }
		
	void offset(const value_type& v) { m_offset = v; }
	value_type offset() const {return m_offset; }
		
	value_type operator () (const value_type& value) const
	{
		internal_type temp = value;
		temp *= m_mult;
		temp /= m_div;
		temp -= m_offset;
		return temp;
	}
	
private:
	value_type m_mult;
	value_type m_div;
	value_type m_offset;
};

#endif