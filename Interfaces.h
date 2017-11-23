/*************************************
*         Copyright 2015 EDF         *
*************************************/
#ifndef INTERFACES_H
#define INTERFACES_H

#include "ILogManager.h"
#include "NamedComp.h"

#include <limits.h>
#include <vector>
#include <complex>
#include <typeinfo>

class CSystem;
class CSystemP;
class IVariable;
class ITransition;
class IState;
class IEquation;
class IIndicator;
class IInequation;
class IAutomaton;
class CMessageBox;
class CComponent;
class CVariable;
class CTransition;

/**Interface de base d'un moniteur.
	Cette classe n'est d�riv�e que par la classe CSysteme.
	Elle permet aux �l�ments monitor�s de conna�tre le moniteur et de lui signaler les changements.
*/
class PYC_PUBLIC IMonitor
{
protected:
	IMonitor(void){}
public:
	/**Demande au moniteur d'enregistrer la valeur de l'�l�ment.*/
	virtual void doMonitor(IMonitored const&elt)=0;
	/**Demande d'ajouter ou d'enlever l'�l�ment du monitoring.*/
	virtual void monitor(IMonitored&elt,bool bMonit)=0;
};

/**Interface de base d'un �l�ment suivi par un moniteur.
	Cette classe est sp�cialis�e par CTransition, CState et CVariable.
	L'interface d�finit localement la gestion du moniteur et de la trace.
*/
class PYC_PUBLIC IMonitored{
	friend class CSystemP;
protected:
	IMonitor*m_Monitor;//!<Moniteur de suivi de l'�l�ment.
	int m_Trace;//!<Niveau de trace de l'�l�ment (0 = pas de trace, 1 = trace en fin de cycle, >1 = trace � chaque affectation).
	bool m_AlwaysMonitored;//!<Si vrai, l'�l�ment est monitor� en permanence
	int m_Id;//!<Id d'identification de l'�l�ment monitor� (d�fini de mani�re unique (si n�gatif : fourni et non modifiable))
	IMonitored():m_Monitor(0),m_Trace(0),m_AlwaysMonitored(false),m_Id(0){}
	virtual~IMonitored(){}
public:
	/**Retourne l'Id de l'�l�ment monitor�.*/
	unsigned int id()const{return (unsigned)(m_Id>0?m_Id:-m_Id);}
	/**D�signe le moniteur de suivi de l'�l�ment (si NULL, l'�l�ment n'est pas suivi), renseigne l'id pr�d�fini et modifie le niveau de monitoring.*/
	virtual void setMonitor(IMonitor*monitor,unsigned id,bool always){if(monitor)monitor->monitor(*this,true);else if(m_Monitor)m_Monitor->monitor(*this,false);m_AlwaysMonitored=(monitor && always);m_Id=monitor?-(int)id:0;}
	/**Retourne le moniteur de suivi de l'�l�ment.*/
	virtual IMonitor*monitor()const{return m_Monitor;}
	/**Modifie le niveau de trace de l'�l�ment (si 0 pas de trace de l'�l�ment).*/
	virtual void setTrace(int trace){m_Trace=trace;}
	/**Retourne le niveau de trace de l'�l�ment.*/
	virtual int trace()const{return m_Trace;}
	/**Modifie l'option de monitoring permanent*/
	void setAlwaysMonitored(bool bAlways){m_AlwaysMonitored=bAlways;}
	/**Retourne vrai si l'�l�ment est monitor� en permanence*/
	int isAlwaysMonitored()const{return m_AlwaysMonitored;}
	/**Retourne le nom simple de l'�l�ment.*/
	virtual char const*basename()const{return 0;}
	/**Retourne le nom complet de l'�l�ment (incluant la hi�rarchie des composants propri�taires de l'�l�ment).*/
	virtual char const*name()const=0;
	/**Retourne le type d'�l�ment*/
	virtual char const*type()const=0;
	/**Retourne la valeur initiale float de l'�l�ment*/
	virtual float fInitValue()const=0;
	/**Retourne l'objet parent de l'�l�ment (dans Pycatshoo, il s'agit d'un CComponent).*/
	virtual CNamedComp*parent()const=0;
	int compare(IMonitored const&other)const;//!<Comparaison en fonction du type et du nom
	bool operator<(IMonitored const&other)const;//!<Comparaison en fonction du type et du nom
};

/**Class de gestion des diff�rentes m�thodes pass�es en param�tres dans Pycatshoo.
La classe permet de g�rer des m�thodes C++ ou des m�thodes Python ou encore des fonctions Python sous la forme d'PyObject.*/
class PYC_PUBLIC CMethod{
protected:
	CNamedComp*m_Comp;
	char*m_Basename;
	bool m_Py;
	static bool st_bUseNameComp;//!<Si vrai, la comparaison des m�thodes prend en compte le nom des composants, sinon, c'est uniquement l'adresse qui compte
public:
	union{
		double(CNamedComp::*m_DblFct)();
		float(CNamedComp::*m_FltFct)();
		int(CNamedComp::*m_IntFct)();
		bool(CNamedComp::*m_BoolFct)();
		void(CNamedComp::*m_VoidFct)();
		double(CNamedComp::*m_DblFctDbl)(double);
		void(CNamedComp::*m_EndFct)(char const*);
		void(CNamedComp::*m_Equation)(IEquation&);
		void(CNamedComp::*m_Inequation)(IInequation&);
		PyObject*m_PyObj;
	};
	CMethod():m_Comp(NULL),m_Basename(NULL),m_Py(false),m_VoidFct(NULL){}
	CMethod(CNamedComp*comp,char const*name,PyObject*pVoid);
	CMethod(CNamedComp*comp,char const*name,double(CNamedComp::*fct)());
	CMethod(CNamedComp*comp,char const*name,float(CNamedComp::*fct)());
	CMethod(CNamedComp*comp,char const*name,int(CNamedComp::*fct)());
	CMethod(CNamedComp*comp,char const*name,bool(CNamedComp::*fct)());
	CMethod(CNamedComp*comp,char const*name,double(CNamedComp::*fct)(double));
	CMethod(CNamedComp*comp,char const*name,void(CNamedComp::*beginFct)());
	CMethod(CNamedComp*comp,char const*name,void(CNamedComp::*endFct)(char const*));
	CMethod(CNamedComp*comp,char const*name,void(CNamedComp::*equation)(IEquation&));
	CMethod(CNamedComp*comp,char const*name,void(CNamedComp::*inequation)(IInequation&));
	CMethod(CMethod const&other):m_Comp(NULL),m_Basename(NULL),m_Py(false){*this=other;}
	virtual ~CMethod();
	char const*basename()const{return m_Basename;}
	char const*compName()const;
	CNamedComp*comp()const{return m_Comp;}
	CMethod&operator=(CMethod const&other);
	int compare(CMethod const&other)const;//!<Comparaison en fonction du nom
	bool operator<(CMethod const&other)const{return compare(other)<0;}
	bool operator==(CMethod const&other)const{return 0==compare(other);}
	static bool mLess(CMethod const*c1,CMethod const*c2){return c1->compare(*c2)<0;}
	double dValue()const;//!<Retourne la valeur double obtenue par appel de dblFct(void) ou d'une fonction Python �quivalente
	virtual float fValue()const;//!<Retourne la valeur float obtenue par appel de fltFct(void) ou d'une fonction Python �quivalente
	int iValue()const;//!<Retourne la valeur int obtenue par appel de intFct(void) ou d'une fonction Python �quivalente
	virtual bool bValue()const;//!<Retourne la valeur bool obtenue par appel de boolFct(void) ou d'une fonction Python �quivalente
	double dValue(double val)const;//!<Retourne la valeur double obtenue par appel de dblFctDbl(double) ou d'une fonction Python �quivalente
	void call()const;//!<Appelle la fonction voidFct(void) ou la fonction Python �quivalente
	void call(char const*)const;//!<Appelle la fonction endFct(void) ou la fonction Python �quivalente
	void call(IEquation&)const;//!<Appelle la fonction equation(IEquation) ou la fonction Python �quivalente
	void call(IInequation&)const;//!<Appelle la fonction inequation(IInequation) ou la fonction Python �quivalente
	static void setUseNameComp(bool bUseCompName);
};

struct SSensitiveMethod:CMethod{
	char m_Sens;
	SSensitiveMethod():m_Sens(0){}
	SSensitiveMethod(CNamedComp*comp,char const*name,void(CNamedComp::*method)(),int sens):CMethod(comp,name,method),m_Sens(sens){}
	SSensitiveMethod(CNamedComp*comp,char const*name,PyObject*pyObj,int sens):CMethod(comp,name,pyObj),m_Sens(sens){}
};

/**Interface d'acc�s � l'it�rateur de s�quence*/
class PYC_PUBLIC ISeqIter{
public:
	virtual~ISeqIter(){}
	virtual void restart()=0;//!<Replace l'it�rateur en d�but de s�quence
	virtual bool atEnd()const=0;//!<Retourne vrai si l'it�rateur a atteint la fin de la s�quence
	virtual bool atBegin()const=0;//!<Retourne vrai si l'it�rateur est en d�but de la s�quence
	virtual void next()=0;//!<Avance au pas d'enregistrement suivant (le temps peut ne pas changer)
	virtual void previous()=0;//!<Recule au pas d'enregistrement pr�c�dent (le temps peut ne pas changer)
	virtual STimeInd const&time()const=0;//!<Retourne l'instant du pas de temps courant
	virtual float value(IMonitored const*obj,bool*exists=NULL)const=0;//!<Retourne la valeur de l'�l�ment obj � l'instant courant s'il existe
	virtual std::vector<IMonitored const*>monitoredElts()const=0;//!<Retourne la liste des �l�ments monitor�s � l'instant courant
};

struct TOperator{
	typedef enum TOp{
		unknown,//!<Inconnu
		ne,//!<Diff�rent
		eq,//!<Egal
		lt,//!<Strictement inf�rieur
		gt,//!<Strictement sup�rieur
		le,//!<Inf�rieur ou �gal
		ge//Sup�rieur ou �gal
	}TOp;
	static char const*opToStr(TOp op);
	static TOp strToOp(char const*str);
	static bool compare(float val1,TOp op,float val2);
	static bool compare(double val1,TOp op,double val2);
};

struct ICondFctFloat{//Interface des fonctors condition avec argument r�el (ce n'est jamais un objet python)
	virtual ~ICondFctFloat(){}
	virtual bool operator()(float val)const{return true;}
};

struct CCondFctFloat:ICondFctFloat{//Fonctor condition construit � partir d'une fonction
	bool (*m_Condition)(float);
	CCondFctFloat(bool (*condition)(float)):m_Condition(condition){}
	inline bool operator()(float val)const{
		return m_Condition(val);
	}
};

class CCondCompFloat:public ICondFctFloat{//Fonctor condition construit � partir d'un op�rateur de comparaison et d'une valeur
	float m_Val;
	TOperator::TOp m_Op;
public:
	CCondCompFloat():m_Val(0),m_Op(TOperator::eq){}
	CCondCompFloat(char const * op, float value):m_Val(value) {
		setOp(op);
	}
	bool operator()(float val)const override;
	bool setOp(char const*op);
	void setValue(float val){m_Val=val;}
	TOperator::TOp op()const{return m_Op;}
	float value()const{return m_Val;}
	char const*opStr()const;
};

/**Interface d'acc�s � une s�quence.
La s�quence permet d'acc�der aux valeurs des �l�ments monitor�s � n'importe quel instant.
Pour suivre le d�roulement d'une s�quence il faut utiliser un it�rateur obtenu avec la m�thode iterator().
*/
class PYC_PUBLIC ISequence{
protected:
	virtual~ISequence(){}
public:
	/**Retourne un it�rateur sur la s�quence.*/
	virtual ISeqIter*iterator()const=0;
	/**Retourne le num�ro de la portion de s�quence*/
	virtual SEQ_ID num()const=0;
	/**Retourne la vraisemblance de la s�quence.*/
	virtual double likelyhood()const=0;
	/**Modifie l'indice de conservation de la s�quence.*/
	virtual void setToKeep(bool toKeep)=0;
	/**Retourne vrai si la s�quence doit �tre conserv�e.*/
	virtual bool toKeep()const=0;
	/**Ajoute une prolongation � la s�quence*/
	virtual void addExtend(ITransition*tr,PyTime date)=0;
	/**Retourne la valeur de l'objet obj � l'instant t.*/
	virtual float value(IMonitored const*obj,PyTime t)const=0;
	/**Calcule les valeurs extr�mes de l'�l�ment monitor� (min et max doivent �tre initialis�s).*/
	virtual void extremeValues(IMonitored const*obj,float&min,float&max)const=0;
	std::vector<float>extremeValues(IMonitored const*obj)const{std::vector<float>v(2,(float)HUGE_VAL);extremeValues(obj,v[0]=(float)-HUGE_VAL,v[1]);return v;}
	/**Calcule les temps de s�jour entre les bornes.*/
	virtual void residenceTimes(IMonitored const*obj,PyTime it,PyTime ft,std::vector<float>const&lims,std::vector<PyTime>&resTs)const=0;
	std::vector<PyTime>residenceTimes(IMonitored const*obj,PyTime it,PyTime ft,std::vector<float>const&lims)const{std::vector<PyTime>v;residenceTimes(obj,it,ft,lims,v);return v;}
	/**Retourne le temps de s�jour satisfaisant la condition.*/
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,ICondFctFloat const&condition)const=0;
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,char const*op,float value)const{
		return residenceTime(obj,it,ft,CCondCompFloat(op,value));
	}
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,bool (*condition)(float))const{
		return residenceTime(obj,it,ft,CCondFctFloat(condition));
	}
	/**Retourne la date de premi�re r�alisation de la condition*/
	virtual PyTime realized(IMonitored const*obj,ICondFctFloat const&condition)const=0;
	virtual PyTime realized(IMonitored const*obj,char const*op,float value)const{
		return realized(obj,CCondCompFloat(op,value));
	}
	PyTime realized(IMonitored const*obj,bool (*condition)(float))const{
		return realized(obj,CCondFctFloat(condition));
	}
	/**Retourne vrai dans real[i] si la condition est satisfaite avant ts[i].*/
	virtual void realized(IMonitored const*obj,ICondFctFloat const&condition,std::vector<PyTime>const&ts,std::vector<bool>&real)const=0;
	void realized(IMonitored const*obj,char const*op,float value,std::vector<PyTime>const&ts,std::vector<bool>&real)const{
		realized(obj,CCondCompFloat(op,value),ts,real);
	}
	void realized(IMonitored const*obj,bool (*condition)(float),std::vector<PyTime>const&ts,std::vector<bool>&real)const{
		realized(obj,CCondFctFloat(condition),ts,real);
	}
	std::vector<bool>realized(IMonitored const*obj,ICondFctFloat const&condition,std::vector<PyTime>const&ts)const{std::vector<bool>v;realized(obj,condition,ts,v);return v;}
	std::vector<bool>realized(IMonitored const*obj,char const*op,float value,std::vector<PyTime>const&ts)const{
		return realized(obj,CCondCompFloat(op,value),ts);
	}
	std::vector<bool>realized(IMonitored const*obj,bool (*condition)(float),std::vector<PyTime>const&ts)const{
		return realized(obj,CCondFctFloat(condition),ts);
	}
	/**Retourne la date d'arr�t de la s�quence*/
	virtual PyTime endTime()const=0;
	/**Retourne la cause d'arr�t de la s�quence*/
	virtual char const*endCause()const=0;
	/**Retourne l'int�grale de la valeur de l'objet obj entre les temps it et ft.*/
	virtual float integralValue(IMonitored const*obj,PyTime it,PyTime ft)const=0;
	/**Retourne dans values les valeurs de l'objet obj aux instants ts.*/
	virtual void values(IMonitored const*obj,std::vector<PyTime>const&ts,std::vector<float>&values)const=0;
	std::vector<float>values(IMonitored const*obj,std::vector<PyTime>const&ts)const{std::vector<float>v;values(obj,ts,v);return v;}
};

/**Interface des filtres de s�quence (ce n'est jamais un objet python)
*/
class PYC_PUBLIC IFilterFct{
	friend class CSystem;
	friend class CSystemP;
	friend class CAnalyser;
public:
	virtual ~IFilterFct(){}
	virtual bool operator()(ISequence const&seq)const=0;
	static IFilterFct*newPatternFilter(char const*pattern);
	static IFilterFct*newConditionFilter(char const*object,PyTime time,char const*op,float val);
};

struct TIndicatorType{
	/**Enum�r� de d�finition du mode de m�morisation des indicateurs.
		Les �l�ments du type peuvent �tre cumul�s.
	*/
	typedef enum EIndicatorType{
		no_value=0,
		mean_values=1,
		std_dev=2,
		all_values=4,
		quantile_le   = 8,
		quantile_gt   = 16,
		distribution=32,
		monitor_values=512
	}EIndicatorType;
};
struct TComputationType{
	/**Enum�r� de d�finition de la valeur calcul�e par l'indicateur.*/
	typedef enum EComputationType{
		simple=0,
		res_time,
		residence_time=res_time,
		nb_visits
	}EComputationType;
};
/**Interface d'acces aux indicateurs.
*/
class PYC_PUBLIC IIndicator:public IMonitored,protected CMethod{
protected:
	IIndicator(){}
	IIndicator(CNamedComp*comp,char const*name,float(CNamedComp::*fct)()):CMethod(comp,name,fct){}
	~IIndicator(){}
public:
	char const*basename()const override{return CMethod::basename();}
	char const*name()const override{return CMethod::basename();}
	virtual void setRestitutions(int type)=0;//!<Modifie les restitutions de l'indicateur
	virtual int restitutions()const=0;//!<Retourne les restitutions de l'indicateur
	virtual void setComputation(TComputationType::EComputationType comp)=0;//!<Modifie la valeur calcul�e par l'indicateur
	virtual TComputationType::EComputationType computation()const=0;//!<Retourne la valeur calcul�e par l'indicateur
	virtual void setPctQuantileLeValue(float fracMin)=0;//!<Modifie le pourcentage de calcul du fractile min
	virtual void setPctQuantileGtValue(float fracMax)=0;//!<Modifie le pourcentage de calcul du fractile max
	virtual float pctQuantileLeValue()const=0;//!<Retourne le pourcentage de calcul du fractile min
	virtual float pctQuantileGtValue()const=0;//!<Retourne le pourcentage de calcul du fractile max
	virtual void setDistribLimits(std::vector<float>lims)=0;//!<Modifie les limites de calcul de la distribution
	virtual std::vector<float>distribLimits()const=0;//!<Retourne les limites de calcul de la distribution
	virtual std::vector<float>const&values(int i)const=0;//!<Retourne toutes les i�me valeurs
	virtual float mean(int i)const=0;//!<Retourne la i�me moyenne
	virtual float stdDev(int i)const=0;//!<Retourne le i�me �cart type
	virtual float quantileLe(int i)const=0;//!<Retourne le i�me fractile min
	virtual float quantileGt(int i)const=0;//!<Retourne le i�me fractile max
	virtual float confInt(float pct,int i)const=0;//!<Retourne le i�me intervalle de confiance � pct%
	virtual std::vector<float>distribution(int i)const=0;//!<Retourne les probabilit�s de la i�me distribution
	virtual std::vector<float>meanDistribution()const=0;//!<Retourne les probabilit�s de la distribution moyenn�e sur tous les instants
	virtual std::vector<float>means()const=0;//!<retourne les moyennes aux diff�rents instants
	virtual std::vector<float>stdDevs()const=0;//!<retourne les �carts types aux diff�rents instants
	virtual std::vector<float>quantilesLe()const=0;//!<retourne les fractiles min aux diff�rents instants
	virtual std::vector<float>quantilesGt()const=0;//!<retourne les fractiles min aux diff�rents instants
	virtual std::vector<float>confInts(float pct)const=0;//!<retourne les intervalles de confiance � pct% aux diff�rents instants
	virtual void setToKeep(bool toKeep)=0;//!<Modifie l'indice de conservation de l'indicateur
	virtual bool toKeep()const=0;//!<Retourne vrai si l'indicateur doit �tre conserv�
};

/**Interface commune � l'ensemble des variables et des r�f�rences.
*/
class PYC_PUBLIC IVarBase:public CNamedComp{
	friend class CSystemState;
	virtual void addSensitiveMethod(SSensitiveMethod const&method)=0;
	CNamedComp*p()const;//!<Juste pour acc�der au parent pour construire la sensitive method
protected:
	CNX_ID m_N;//!<Nombre maximum de liens accept�s par la variable.
	IVarBase(char const*name,CNamedComp*parent):CNamedComp(name,parent),m_N(0xffffffff){}
public:
	/**Retourne le type de la valeur courante de la variable.*/
	virtual std::type_info const&typeInfo()const{return typeid(bool);}
	/**Retourne le nombre maximum de connexion sur la variable.*/
	virtual CNX_ID cnctMax()const{return m_N;}
	/**Ajoute une m�thode sensible aux modifications de la variable.
	@param name nom de la m�thode
	@param method m�thode sensible � ajouter
	@param sens sens de sensibilit� de la m�thode (0 tous, 1 accroissement, -1 r�duction)
	*/
	template<typename Type>void addSensitiveMethod(char const*name,void(Type::*method)(),int sens=0){addSensitiveMethod(SSensitiveMethod(p(),name,static_cast<void(CNamedComp::*)()>(method),sens));}
	void addPySensitiveMethod(char const*name,PyObject*pVoid,int sens){addSensitiveMethod(SSensitiveMethod(p(),name,pVoid,sens));}
	/**Supprime la m�thode de nom name des m�thodes sensible aux modifications de la variable*/
	virtual void removeSensitiveMethod(char const*name)=0;
	virtual void clearSensitiveMethod()=0;
	/**Modifie le nombre maximum de connexion sur la variable*/
	void setCnctMax(CNX_ID N){m_N=N;}
};

/**Interface de gestion des variables.
	Les variables peuvent �tre des variables d'�tat ou des automates ou des �tats.
*/
class PYC_PUBLIC IVariable : public IVarBase,public IMonitored{
	void value(bool&res)const{res=bValue();}
	void value(int&res)const{res=iValue();}
	void value(float&res)const{res=fValue();}
	void value(double&res)const{res=dValue();}
	void value(std::complex<double>&res)const{res=cValue();}
	void value(std::string&res)const{res=sValue();}
protected:
	IVariable():IVarBase(NULL,NULL){}
	IVariable(CNamedComp&parent,char const*name):IVarBase(name,&parent){}
public:
	char const*basename()const override{return CNamedComp::basename();}
	char const*name()const override{return CNamedComp::name();}
	CNamedComp*parent()const override{return CNamedComp::parent();}
	virtual CVariable const*variable()const{return NULL;}
	/**Modifie la r�initialisation de la variable*/
	virtual void setReinitialized(bool toReinit){ILogManager::glLogManager().throwError("M�thode inutilisable sur cette classe");}
	/**Retourne la r�initialisation de la variable*/
	virtual bool isReinitialized()const{return false;}
	/**Modifie la gestion de l'interpolation des valeurs de la variable*/
	virtual void setInterpolated(bool interp);
	/**Retourne vrai si les valeurs de la variable doivent �tre interpol�es*/
	virtual bool isInterpolated()const{return false;}
	/**Modifie la d�riv�e de la valeur pour les ODE*/
	virtual void setDvdtODE(double dvdt)=0;
	/**Modifie le fait que la variable soit modifiable de l'ext�rieur*/
	virtual void setModifiable(bool res)=0;
	/**Retourne la d�riv�e de la valeur de la variable (uniquement pour les variables double)*/
	virtual double derivative(double dt)const=0;
	/**Retourne le pas de calcul de la d�riv�e*/
	virtual double dtDerivative(double dt)const{return derivative(dt);/*Pour obtenir le message d'erreur par d�faut*/}
	/**D�signe la variable dont la variable courante est l'int�grale.*/
	virtual void integrate(IVariable*var);
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de complexe.*/
	virtual void setValue(std::complex<double>val)=0;
	inline void setCValue(std::complex<double>val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de double.*/
	virtual void setValue(double val)=0;
	inline void setDValue(double val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de float.*/
	virtual void setValue(float val)=0;
	inline void setFValue(float val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de int.*/
	virtual void setValue(int val)=0;
	inline void setIValue(int val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de bool.*/
	virtual void setValue(bool val)=0;
	inline void setBValue(bool val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lanc�e) sous forme de string.*/
	virtual void setValue(char const*val)=0;
	inline void setSValue(char const*val){setValue(val);}
	/**Retourne la valeur courante sous forme de complexe.*/
	virtual std::complex<double> cValue()const=0;
	/**Retourne la valeur courante sous forme de double.*/
	virtual double dValue()const=0;
	/**Retourne la valeur courante sous forme de float.*/
	virtual float fValue()const=0;
	/**Retourne la valeur courante sous forme de int.*/
	virtual int iValue()const=0;
	/**Retourne la valeur courante sous forme de bool.*/
	virtual bool bValue()const=0;
	/**Retourne la valeur courante sous forme de string.*/
	virtual std::string sValue()const=0;
	/**Retourne la valeur courante.*/
	template<typename T>T value()const{
		T res;
		value(res);
		return res;
	}
	/**Retourne la valeur initiale sous forme de complexe.*/
	virtual std::complex<double> cInitValue()const=0;
	/**Retourne la valeur initiale sous forme de double.*/
	virtual double dInitValue()const=0;
	/**Retourne la valeur initiale sous forme de float.*/
	virtual float fInitValue()const=0;
	/**Retourne la valeur initiale sous forme de int.*/
	virtual int iInitValue()const=0;
	/**Retourne la valeur initiale sous forme de bool.*/
	virtual bool bInitValue()const=0;
	/**Retourne la valeur initiale sous forme de string.*/
	virtual std::string sInitValue()const=0;
	/**Retourne 0 si la valeur courante est indentique � la valeur initiale et + ou -1 suivant le sens de variation.*/
	virtual int valueChanged()const=0;
	/**Applique la valeur de red�marrage � la valeur courante.*/
	virtual void reset()=0;
	/**Demande la gestion du monitoring de la variable.*/
	virtual void monitorValue()const=0;
};

/**Interface de gestion des r�f�rences � des variables.
	Les r�f�rences sont le reflet de variables situ�es dans d'autres composants.
*/
class PYC_PUBLIC IReference : public IVarBase{
	void value(bool&res,CNX_ID i)const{res=bValue(i);}
	void value(int&res,CNX_ID i)const{res=iValue(i);}
	void value(float&res,CNX_ID i)const{res=fValue(i);}
	void value(double&res,CNX_ID i)const{res=dValue(i);}
	void value(std::complex<double>&res,CNX_ID i)const{res=cValue(i);}
	void value(std::string&res,CNX_ID i)const{res=sValue(i);}
	using IVarBase::typeInfo;
protected:
	IReference(CNamedComp&parent,char const*name):IVarBase(name,&parent){}
public:
	/**Retourne le type de la i�me connexion*/
	virtual std::type_info const&typeInfo(CNX_ID i)const=0;
	/**Retourne le nombre de valeurs connues par la r�f�rence (connexions r�elles ou valeurs fixes).*/
	virtual CNX_ID cnctCount()const=0;
	virtual CNX_ID nbCnx()const{return cnctCount();}
	/**Retourne la valeur courante de la i�me connexion sous forme de complexe.*/
	virtual std::complex<double>cValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la i�me connexion sous forme de double.*/
	virtual double dValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la i�me connexion sous forme de float.*/
	virtual float fValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la i�me connexion sous forme de int.*/
	virtual int iValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la i�me connexion sous forme de bool.*/
	virtual bool bValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la i�me connexion cast�e au type T.*/
	template<typename T>T value(CNX_ID i)const{
		T res;
		value(res,i);
		return res;
	}
	/**Retourne la valeur courante de la i�me connexion sous forme de string.*/
	virtual std::string sValue(CNX_ID i)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de complexe.*/
	virtual CNX_ID cValues(std::complex<double>*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de double.*/
	virtual CNX_ID dValues(double*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de float.*/
	virtual CNX_ID fValues(float*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme d'entiers.*/
	virtual CNX_ID iValues(int*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de bool�ens.*/
	virtual CNX_ID bValues(bool*tab)const=0;
	/**Retourne la r�f�rence associ�e � la i�me connexion.*/
	virtual IReference*reference(CNX_ID i)const=0;
	/**Retourne la variable associ�e � la i�me connexion.*/
	virtual IVariable*variable(CNX_ID i)const=0;
	/**Retourne la IVarBase associ�e � la i�me connexion.*/
	virtual IVarBase*varBase(CNX_ID i)const=0;
	/**Prend une quantit� res de ressource � la i�me variable connect�e.*/
	virtual void takeResource(CNX_ID i,double res)=0;
	/**Restitue une quantit� res de ressource � la i�me variable connect�e.*/
	virtual void restoreResource(CNX_ID i,double res)=0;
	/**Modifie la valeur de la i�me variable connect�e.*/
	virtual void setValue(CNX_ID i,double val)=0;
	/**Modifie la d�riv�e de la i�me variable connect�e (uniquement si c'est une variable double).*/
	virtual void setDvdt(CNX_ID i,double val)=0;
	/**Lie la r�f�rence � une autre r�f�rence ou variable vb*/
	virtual void bind(IVarBase&vb)=0;
	/**D�lie la r�f�rence de la r�f�rence ou de la variable vb*/
	virtual void unbind(IVarBase&vb)=0;
	/**D�lie la i�me connexion*/
	virtual void unbind(CNX_ID i)=0;
	/**Calcule un ou sur l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual bool orValue(bool def=false)const=0;
	/**Calcule un et sur l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual bool andValue(bool def=true)const=0;
	/**Calcule une somme de l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual double sumValue(double def=0)const=0;
	/**Calcule un produit l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual double productValue(double def=1)const=0;
	/**Remplit le vecteur avec les indices qui v�rifient la condition*/
	virtual int valuesThat(ICondFctFloat const&cf, std::vector<int> & indexes)const=0;
	virtual int valuesThat(bool(condition)(float), std::vector<int> & indexes)const{return valuesThat(CCondFctFloat(condition),indexes);}
};

/**Interface de gestion des bo�tes de messages.
*/
class PYC_PUBLIC IMessageBox:public CNamedComp{
protected:
	IMessageBox(CNamedComp&parent,char const*name):CNamedComp(name,&parent){}
public:
	/**Ajoute une variable � connecter.*/
	virtual void addExport(IVarBase*p,char const*aliasName=NULL)=0;
	/**Retourne la variable export�e sous l'alias aliasName (exception si c'est une r�f�rence).*/
	virtual IVariable*exportedVariable(char const*aliasName)const=0;
	/**Ajoute une r�f�rence � connecter et d�finit le nom de la variable � lui associer.*/
	virtual void addImport(IReference*p,char const*varName=NULL)=0;
	/**Ajoute une r�f�rence � connecter facultativement et d�finit le nom de la variable � lui associer et sa valeur par d�faut s'il n'est pas connect�.*/
	virtual void addOptionalImport(IReference*p,int type, double defaultValue, char const*alias)=0;
	//*Retourne vrai si la connection peut �tre effectu�e*/
	virtual bool canConnectTo(IMessageBox&other)const=0;
	//*Retourne vrai si la connection existe d�j�*/
	virtual bool isConnectedTo(IMessageBox&other)const=0;
	/**Connecte la bo�te de messages � une autre bo�te de messages.*/
	virtual void connectTo(IMessageBox&other,double weight)=0;
	void connectTo(IMessageBox&other){connectTo(other,0);}//Pour le python
	/**deconnecte la bo�te de messages d'une autre bo�te de messages.*/
	virtual void disconnectFrom(IMessageBox&other)=0;
	/**deconnecte la bo�te de messages des autres bo�tes de messages.*/
	virtual void disconnect()=0;
	/**Retourne le nombre de connexions*/
	virtual CNX_ID cnctCount()const=0;
	/**Retourne le poids de la i�me connection*/
	virtual double weight(CNX_ID i)const=0;
	/**Retourne le i�me composant connect�*/
	virtual CComponent*component(CNX_ID i)const=0;
	/**Retourne la i�me bo�te de message connect�e*/
	virtual IMessageBox*cnct(CNX_ID i)const=0;
	/**Retourne sous forme de complexe la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual std::complex<double> cValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de double la valeur la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual double dValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de float la valeur la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual float fValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de int la valeur la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual int iValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de bool la valeur la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual bool bValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de string la valeur la valeur de la i�me variable connect�e export�e sous l'alias aliasName.*/
	virtual std::string sValue(CNX_ID i,char const*aliasName)const=0;
};

struct SBindingMM;
struct SBindingM;
struct SBinding;
/**Interface des foncteurs de modification de loi de probabilit�
Le foncteur modifie la valeur "normale" qui est fournie en iniVal.
*/
struct IDistLawModifier{
	virtual double operator()(double iniVal)const=0;
	~IDistLawModifier(){}
};
struct TLawType{
	/**D�finit les types de loi pr�d�finis dans le coeur de Pycatshoo*/
	typedef enum ELawType{
		inst,//!<Loi instantan�e
		expo,//!<Loi exponentielle
		cstt,//!<Loi temps constant
		defer=cstt,
		cstd//!<Loi date constante
	}ELawType;
};
/**Interface de gestion des lois de probabilit�.
*/
class PYC_PUBLIC IDistLaw{
	friend class CDistLawInst;
	friend class ITransition;
	friend class CTransition;
	LAWP_ID m_NbParam;
	CNamedComp&m_Parent;
	SBindingMM*m_Parameters;//!<D�finition premi�re des param�tres permettant de mettre � jour les valeurs num�riques
	double*m_ParamValues;//!<Valeurs des param�tres utilis�es pour le calcul du temps d'attente
	void setParameter(double(CNamedComp::*fct)(),LAWP_ID place);
	void reset();//!<Replace les valeurs initiales
	void insertAt(LAWP_ID place,bool bAdd=false);//!<Cr�e une place en place
	void incRef();
	unsigned int m_NbCppRef;//!<Nombre de r�f�rences d'un objet C++
	PyObject*m_Self;//!<R�f�rence � l'objet python correspondant (NULL, si c'est un objet purement C++).
	IDistLaw(IDistLaw const&);//Interdit
	IDistLaw&operator=(IDistLaw const&);//Interdit
protected:
	virtual~IDistLaw(void);
	IDistLaw(PyObject*self,CNamedComp&parent);
public:
	explicit IDistLaw(CNamedComp&parent);
	/**Donne le nom du type de distribution*/
	virtual char const*name()const{return NULL;}
	/**Retourne le type de loi (parmi TLawType pour les type standard)*/
	virtual int type()const{return -1;}
	void deleteDLaw();//!<Detruit la loi sauf si m_Self est non null
	/**D�finit le nombre de param�tres de la loi (par d�faut 1)*/
	void setNbParameters(LAWP_ID nb){if(nb>m_NbParam)insertAt(nb-1);else m_NbParam=nb;}
	/**Insert un param�tre sous forme de valeur.*/
	void insertParameter(double value,LAWP_ID place=0);
	/**Insert un param�tre sous forme de variable.*/
	void insertParameter(IVariable&var,LAWP_ID place=0);
	/**D�finit un param�tre par connexion d'une variable.*/
	void setParameter(IVariable&var,LAWP_ID place=0);
	/**D�finit un param�tre par une m�thode*/
	template<typename Type>void setParameter(double (Type::*fct)(),LAWP_ID place=0){setParameter(static_cast<double (CNamedComp::*)()>(fct),place);}
	/**Modifie la valeur d'un param�tre.*/
	void setParameter(double value,LAWP_ID place=0);
	/**D�finit un param�tre par une fonction Python*/
	void setPyParameter(char const*name,PyObject*pyObj,LAWP_ID place);
	/**D�finit un modificateur pour un param�tre existant*/
	void setParameterModifier(IDistLawModifier const*dlModif,LAWP_ID place);
	/**Retourne la valeur d'un param�tre*/
	virtual double parameter(LAWP_ID place=0)const{return m_ParamValues[place];}
	/**Retourne le temps d'attente tir� al�atoirement avec les param�tres courants de telle sorte que le d�lai soit sup�rieur � dMin*/
	virtual PyTime delayBeyond(PyTime dMin)const{return dMin;}
	/**Retourne le temps d'attente tir� al�atoirement avec les param�tres courants*/
	virtual PyTime delay()const{return delayBeyond(0.);}
	/**Retourne le temps minimal d'attente*/
	virtual PyTime delayMin()const{return 0.;}
	/**Retourne le temps minimal d'attente*/
	virtual PyTime delayMax()const{return HUGE_VAL;}
	/**Retourne la valeur de la densit� de probabilit�*/
	virtual double density(PyTime t)const{return 0.;}
	/**Retourne la valeur de la fonction de r�partition*/
	virtual double distribution(PyTime t)const{return 0.;}
	/**Retourne l'indexe de la transition de sortie tir� al�atoirement avec les param�tres courants (� partir de 0 jusqu'� nbIndex-1)*/
	virtual TGT_ID index()const{return 0;}
	/**Retourne le nombre de param�tres de la loi*/
	inline LAWP_ID nbParam()const{return m_NbParam;}
	/**Retourne le nombre de sorties g�r�es par la loi.*/
	virtual TGT_ID nbIndex()const{return 1;}
	/**Retourne faux si les param�tres ont chang�*/
	bool upToDate();
	/**Construit une des lois pr�d�finies*/
	static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,IVariable&var);
	/**Construit une des lois pr�d�finies*/
	static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,double param);
	/**Construit une des lois pr�d�finies*/
	template<typename Type>static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,double (Type::*fct)()){IDistLaw*dl=newLaw(parent,type,0);dl->setParameter(fct);return dl;}
	static IDistLaw*newPyLaw(CNamedComp&parent,TLawType::ELawType type,char const*name,PyObject*pVoid){IDistLaw*dl=newLaw(parent,type,0);dl->setPyParameter(name,pVoid,0);return dl;}
	static IDistLaw*newPyLaw(CNamedComp&parent,TLawType::ELawType type,char const*name){return newPyLaw(parent,type,name,NULL);}
};

struct TModificationMode{
	/**Enum�r� de d�finition de l'aspect modifiable de la transition.*/
	typedef enum EModificationMode{
		not_modifiable,//!<Transition non modifiable
		not_modif=not_modifiable,
		discrete_modification,//!<Transition modifiable � chaque pas
		modif=discrete_modification,
		continuous_modification,//!<Transition modifiable contin�ment (dans les ODE)
		cont_modif=continuous_modification
	}EModificationMode;
};
struct TTransType{
	/**Enum�r� de d�finition de type de franchissement de la transition.*/
	typedef enum ETransType{
		trans=1,//!<Transition standard
		fault=2,//!<D�faillance
		rep=3//!<R�paration
	}ETransType;
	static char const*st_Names[];
};
/**Interface de gestion des transitions d'un �tat � un autre.
	Une transition permet de contr�ler le passage d'un �tat � un autre.
	Une transition g�re :
	- une condition par l'interm�diaire d'une r�f�rence m_Condition,
	- des param�tres de loi par l'interm�diaire de la r�f�rence m_Parameters.
	Une transition conna�t son �tat de d�part et ses �tats d'arriv�e potentiels.
	Il ne peut y avoir plusieurs �tats d'arriv�e que pour une transition INS.

	Les mod�les actuellement reconnus sont :
	- INS : les param�tres sont les proba de passage dans les �tats cibles (le dernier param�tre peut �tre ignor�)
	- EXP : l'unique param�tre est le taux de passage par unit� de temps (seconde)
	- TC  : l'unique param�tre est la dur�e d'attente en seconde

	La transition peut �tre qualif�e de :
	- trans : transition standard,
	- def : d�faillance,
	- rep : r�paration.

	La transition peut �tre :
	- interruptible (si la condition est invalid�e pendant l'attente, l'attente est interrompue),
	- modifiable (si les param�tres sont modifi�s pendant l'attente, celle-ci est recalcul�e).
*/
class PYC_PUBLIC ITransition : public CNamedComp,public IMonitored{
protected:
	SBindingM&m_Condition;//!<D�finition de la condition de franchissement de la transition
	bool m_NCond;//!<Si vrai, la condition est utilis�e � l'envers
	bool m_NCondInit;//!<Si vrai, la condition est initialement utilis�e � l'envers
	bool m_Interruptible;//!<Si vrai, la transition peut �tre interrompue si la condition n'est plus valide (par d�faut faux)
	TModificationMode::EModificationMode m_Modifiable;//!<Si 1 la transition peut �tre modifi�e apr�s son lancement, si 2 elle est continuement modifiable (int�gration) (par d�faut non modifiable)
	IDistLaw*m_DistLaw;//!<Loi de distribution des probabilit�s
	virtual void addSensitiveMethod(SSensitiveMethod const&method)=0;
	void setCondition(bool(CNamedComp::*fct)(),bool negate);
	ITransition(CNamedComp&parent,char const*name);
	virtual~ITransition();
public:
	char const*basename()const override{return CNamedComp::basename();}
	char const*name()const override{return CNamedComp::name();}
	CNamedComp*parent()const override{return CNamedComp::parent();}
	/**Retourne la loi de probabilit� de la transition*/
	IDistLaw*distLaw()const{return m_DistLaw;}
	/**Modifie la loi de probabilit� de la transition*/
	void setDistLaw(IDistLaw*distLaw);
	/**Modifie la loi de probabilit� de la transition*/
	IDistLaw*setDistLaw(TLawType::ELawType type,double param){setDistLaw(IDistLaw::newLaw(*parent(),type,param));return distLaw();}
	IDistLaw*setDistLaw(TLawType::ELawType type,IVariable&param){setDistLaw(IDistLaw::newLaw(*parent(),type,param));return distLaw();}
	template<typename Type>IDistLaw*setDistLaw(TLawType::ELawType type,double(Type::*fct)()){setDistLaw(IDistLaw::newLaw(*parent(),type,fct));return distLaw();}
	IDistLaw*setPyDistLaw(TLawType::ELawType type,char const*name,PyObject*pVoid){setDistLaw(IDistLaw::newPyLaw(*parent(),type,name,pVoid));return distLaw();}
	IDistLaw*setPyDistLaw(TLawType::ELawType type,char const*name){setPyDistLaw(type,name,NULL);return distLaw();}
	/**Modifie le caract�re interruptible de la transition (par d�faut faux).*/
	void setInterruptible(bool interruptible){m_Interruptible=interruptible;}
	/**Retourne le crat�re interruptible de la transition.*/
	bool interruptible()const{return m_Interruptible;}
	/**Modifie le caract�re modifiable de la transition (par d�faut faux).*/
	void setModifiable(TModificationMode::EModificationMode modifiable){m_Modifiable=modifiable;}
	/**Retourne le crat�re modifiable de la transition.*/
	TModificationMode::EModificationMode modifiable()const{return m_Modifiable;}
	/**Retourne l'�tat de d�part de la transition*/
	virtual IState const*startState()const=0;
	/**Retourne le nombre de cibles de la transition*/
	virtual TGT_ID targetCount()const=0;
	/**Insert un �tat cible � la transition et d�finit le type de la transition vers cet �tat.*/
	virtual void insertTarget(IState*end,int type,int place)=0;
	/**Ajoute un �tat cible � la transition et d�finit le type de la transition vers cet �tat.*/
	virtual void addTarget(IState*end,int type=TTransType::trans)=0;
	/**Retourne l'�tat cible d'indice i.*/
	virtual IState const*getTarget(TGT_ID i)const=0;
	/**Retourne le type de franchissement effectu� pour atteindre le i�me �tat cible.*/
	virtual int getTargetType(TGT_ID ind)const=0;
	/**Inhibe ou d�sinhibe les sorties vers l'�tat*/
	virtual void inhibateTarget(IState*st,bool inhibate)=0;
	/**Inhibe ou d�sinhibe la sortie vers les cibles de type type*/
	virtual void inhibateTarget(int type,bool inhibate)=0;
	/**Renseigne la condition par connexion � une variable ou � un �tat.*/
	void setCondition(IVariable&var,bool negate=false);
	void setCondition(IVariable*var,bool negate=false){setCondition(*var,negate);}
	/**Renseigne la condition par une fonction bool�enne.*/
	template<typename FctType> void setCondition(FctType fct,bool negate=false){setCondition(static_cast<bool(CNamedComp::*)()>(fct),negate);}
	/**Renseigne la condition par une fonction Python.*/
	void setPyCondition(char const*name,PyObject*pyObj,bool negate=false);
	/**Renseigne la condition par une m�thode Python nomm�e.*/
	void setPyCondition(char const*name,bool negate=false){setPyCondition(name,NULL,negate);}
	/**Renseigne la condition par une valeur bool�enne.*/
	void setCondition(bool val);
	/**Retourne la valeur actuelle de la condition.*/
	bool getCondition()const;
	/**Retourne la date de franchissement pr�vue (ou -1 s'il n'y a pas de date pr�vue).*/
	virtual PyTime endTime()const=0;
	/**Retourne la date de d�cision d'activation*/
	virtual PyTime startTime()const=0;
	/**Modifie le d�lai d'attente*/
	virtual void setDelay(PyTime delay)=0;
	/**Retourne l'indice de l'�tat de sortie choisi ou -1 sinon*/
	virtual int indOutState()const=0;
	/**Modifie l'indice de l'�tat de sortie*/
	virtual void setIndOutState(unsigned ind)=0;
	/**Ajoute une m�thode sensible au tirage de la transition*/
	template<typename FctType>void addSensitiveMethod(char const*name,FctType method,int outState=-1){addSensitiveMethod(SSensitiveMethod(parent(),name,static_cast<void(CNamedComp::*)()>(method),outState));}
	void addPySensitiveMethod(char const*name,PyObject*pVoid,int outState=-1);
	/**Supprime une m�thode sensible au tirage de la transition*/
	virtual void removeSensitiveMethod(char const*name)=0;
	/**Retourne l'indice de l'�tat de sortie atteint lors du tirage de la transition (ou -1 si la transition n'a pas �t� tir�e)*/
	virtual int firedState()const=0;
	/**Force � recalculer le d�lai d'attente*/
	virtual void invalidate()=0;
	/**Modifie le masque des �tats de sortie � monitorer*/
	virtual void setMonitoredOutStateMask(char const*mask)=0;
	/**Retourne le masque des �tats de sortie � monitorer*/
	virtual char const*monitoredOutStateMask()const=0;
};

/**Interface de gestion des �tats.
	Les �tats permettent de construire le diagramme d'�tats d'un composant � l'aide des transitions.
	Un �tat peut �tre actif ou non. Un �tat actif active ses transitions de sortie.
	Un �tat peut �tre export� dans une bo�te de messages comme une vzariable.
*/
class PYC_PUBLIC IState:public virtual IVariable{
public:
	/**Ajoute une transition de sortie de l'�tat.*/
	virtual ITransition*addTransition(char const*name)=0;
	/**Retourne vrai si l'�tat est actif.*/
	virtual bool isActive()const{return bValue();}
	/**Retourne la date de sortie de l'�tat la plus proche (en seconde).*/
	virtual double exitTime()const=0;
	/**Retourne l'automate de l'�tat.*/
	virtual IAutomaton*automaton()const=0;
	/**Retourne l'indice de l'�tat (doit �tre unique dans l'automate).*/
	virtual unsigned int index()const=0;
	/**Retourne la liste des transitions au d�part de l'�tat.*/
	virtual std::vector<ITransition*>transitions()const=0;
};

/**Interface de gestion des automates d'�tats.
	Les automates g�rent le diagramme des �tats d'un composant.
	Un automate peut restituer l'�tat courant ou sont index et �tre export� dans une bo�te de messages comme une variable.
*/
class PYC_PUBLIC IAutomaton:public virtual IVariable{
public:
	virtual size_t nbStates()const=0;//Retourne le nombre d'�tats de l'automate
	virtual std::vector<IState*>states()const=0;//!<Retourne la liste des �tats de l'automate
	/**Modifie l'�tat initial de l'automate.*/
	virtual void setInitState(IState*st)=0;
	/**Retourne l'�tat initial ou NULL.*/
	virtual IState*initState()const=0;
	/**Retourne l'indice de l'�tat courrent ou -1.*/
	virtual int currentIndex()const=0;
	/**Retourne l'�tat courant ou NULL.*/
	virtual IState*currentState()const=0;
	/**Ajoute un �tat � l'automate.*/
	virtual IState*addState(char const*name,int index)=0;
};

/**Interface des conditions d'�tapes (ce n'est jamais un objet python)
*/
struct ICondFct{
	virtual bool operator()()const{return true;}
};

class CCondFct:public ICondFct{
public:
	explicit CCondFct(bool (*condition)()):m_Condition(condition){}
	bool (*m_Condition)();
	inline bool operator()()const{
		return m_Condition();
	}
};

/**Interface de manipulation des �tapes.*/
struct IStep{
public:
	/**Retourne le nom de l'�tape*/
	virtual char const*name()const=0;
	/**Ajoute une m�thode � l'�tape*/
	virtual void addMethod(CNamedComp const*cmp,char const*methodName)=0;
	/**Modifie la condition de l'�tape.
	L'�tape prend possession du fonctor condition*/
	virtual void setCondition(ICondFct*fct)=0;
	/**Modifie la condition de l'�tape.*/
	void setCondition(bool(*fct)()){setCondition(new CCondFct(fct));}
	/**Retourne vrai si la condition de l'�tape est satisfaite*/
	virtual bool condition()const=0;
};

struct TSchemaType{
	/**Enum�r� de choix de l'algorithme d'int�gration.
	*/
	typedef enum ESchemaType{
		euler=1,//!<Algorithme d'euler
		runge_kutta4,//!<Runge kutta d'ordre 4
		runge_kutta_cash_karp54,//!<Runge kutta d'ordre 4/5
		runge_kutta_dopri5,//!<Runge kutta d'ordre 5
		runge_kutta_fehlberg78,//!<Runge kutta fehlberg
		modified_midpoint,//!<Point milieu modifi�
		controlled_rk_cash_karp54,//!<Runge kutta � pas variable
		controlled_rk_dopri5,//!<Runge kutta � pas variable
		controlled_rk_fehlberg78,//!<Runge kutta � pas variable
		controlled_bulirsch_stoer,//!<Bulirsch Stoer naturellement � pas variable
		dense_rk_dopri5,//!<Runge kutta dense et � pas variable
		dense_bulirsch_stoer//!<Bulirsch Stoer dense et � pas variable
	}ESchemaType;
	static char const*st_Names[];
};
/**Interface de gestion des solveurs ODE de Pycatshoo*/
class PYC_PUBLIC IPDMPManager:public CNamedComp
{
protected:
	IPDMPManager(CSystem&system,char const*name):CNamedComp(system,name),m_Trace(0){}
	typedef struct TODEFct:CMethod{
		int m_Order;
		TODEFct():m_Order(0){}
		TODEFct(CNamedComp*comp,char const*name,PyObject*pVoid,int order):CMethod(comp,name,pVoid),m_Order(order){}
		TODEFct(CNamedComp*comp,char const*name,void(CNamedComp::*ODEFct)(),int order):CMethod(comp,name,ODEFct),m_Order(order){}
	}TODEFct;

	virtual void addEquationMethod(TODEFct const&fct)=0;//!<Ajoute une fonction de calcul des d�riv�es des variables
	virtual void addBeginMethod(CMethod const&fct)=0;//!<Ajoute une fonction � appeler avant l'int�gration
	virtual void addEndMethod(CMethod const&fct)=0;//!<Ajoute une fonction � appeler apr�s l'int�gration
	virtual void addCondition(CMethod const&fct)=0;//!<Ajoute une fonction de calcul de condition d'arr�t de l'int�gration
	virtual void addCondition(CTransition*tr)=0;//!<Ajoute une transition utilis�e pour sa condition
	int m_Trace;
public:
	/**Modifie l'algorithme d'int�gration (par d�faut Runge kutta d'ordre 4).*/
	virtual void setSchema(TSchemaType::ESchemaType schema)=0;
	/**Retourne l'algorithme d'int�gration utilis�.*/
	virtual TSchemaType::ESchemaType schema()const=0;
	/**Modifie le pas d'int�gration (par d�faut 0.01).*/
	virtual void setDt(double dt)=0;
	virtual double getDt()const=0;
	/**Modifie le pas d'int�gration minimum (par d�faut 0).*/
	virtual void setDtMin(double dtMin)=0;
	virtual double getDtMin()const=0;
	/**Modifie le pas d'int�gration maximum (par d�faut l'infini).*/
	virtual void setDtMax(double dtMax)=0;
	virtual double getDtMax()const=0;
	/**Modifie la pr�cision de recherche des intersections (par d�faut 0.001).*/
	virtual void setDtCond(double dt)=0;
	virtual double getDtCond()const=0;
	/**Modifie la p�riodicit� de m�morisation des variables (par d�faut 0.1).*/
	virtual void setDtMem(double dt)=0;
	virtual double getDtMem()const=0;
	/**Modifie l'option de stockage syst�matique des variables (par d�faut false).*/
	virtual void setAlwaysMem(bool bAlways)=0;
	virtual bool alwaysMem()const=0;
	/**Ajoute une variable � g�rer par le syst�me d'ODE.*/
	virtual void addODEVariable(IVariable&var)=0;
	/**Ajoute une variable � calculer en m�me temps que le syst�me d'ODE.*/
	virtual void addExplicitVariable(IVariable&var)=0;
	/**Ajoute une fonction � appeler avant l'int�gration*/
	template<typename Type> void addBeginMethod(char const*name,Type&comp,void(Type::*beginFct)()){addBeginMethod(CMethod(&comp,name,static_cast<void(CNamedComp::*)()>(beginFct)));}
	/**Ajoute une fonction � appeler apr�s l'int�gration*/
	template<typename Type> void addEndMethod(char const*name,Type&comp,void(Type::*endFct)(char const*)){addEndMethod(CMethod(&comp,name,static_cast<void(CNamedComp::*)(char const*)>(endFct)));}
	/**Ajoute une fonction d'�quation diff�rentielle*/
	template<typename Type> void addEquationMethod(char const*name,Type&comp,void(Type::*odeFct)(),int order=0){addEquationMethod(TODEFct(&comp,name,static_cast<void(CNamedComp::*)()>(odeFct),order));}
	/**Ajoute une fonction de calcul de condition*/
	template<typename Type> void addCondition(char const*name,Type&comp,double(Type::*condFct)()){addCondition(CMethod(&comp,name,static_cast<double(CNamedComp::*)()>(condFct)));}
	template<typename Type> void addBoundaryCheckerMethod(char const*name,Type&comp,double(Type::*condFct)()){addCondition(CMethod(&comp,name,static_cast<double(CNamedComp::*)()>(condFct)));}
	/**Ajoute une transition utilis�e pour sa condition*/
	void addTransitionCondition(ITransition&tr);
	void addPyBeginMethod(char const*name,CComponent&comp,PyObject*pVoid);
	void addPyBeginMethod(char const*name,CComponent&comp){addPyBeginMethod(name,comp,NULL);}
	void addPyEndMethod(char const*name,CComponent&comp,PyObject*pVoid);
	void addPyEndMethod(char const*name,CComponent&comp){addPyEndMethod(name,comp,NULL);}
	void addPyEquationMethod(char const*name,CComponent&comp,PyObject*pVoid,int order=0);
	void addPyEquationMethod(char const*name,CComponent&comp,int order=0){addPyEquationMethod(name,comp,NULL,order);}
	void addPyCondition(char const*name,CComponent&comp,PyObject*pVoid);
	void addPyCondition(char const*name,CComponent&comp){addPyCondition(name,comp,NULL);}
	/**Modifie la trace du PDMP (juste 0 ou non nul)*/
	void setTrace(int trace){m_Trace=trace;}
	int trace()const{return m_Trace;}
	/**Retourne vrai si la variable fait partie des variable explicites du PDMP*/
	virtual bool isExplicitVariable(IVariable const*var)const=0;
	/**Retourne vrai si la variable fait partie des variable d'ODE du PDMP*/
	virtual bool isODEVariable(IVariable const*var)const=0;
};

/**Classe de d�finition d'une �quation*/
class PYC_PUBLIC IEquation{
protected:
	virtual~IEquation(){}
public:
	/**Ajoute un coefficient associ� � une variable*/
	virtual void setCoefficient(IVariable*var,double cR,double cI=0)=0;
	/**D�finit la constate du second membre*/
	virtual void setConstant(double cR,double cI=0)=0;
};

/**Classe de d�finition et de r�solution de syst�mes d'�quations lin�aires.
*/
class PYC_PUBLIC ISLEManager:public CNamedComp{
protected:
	ISLEManager(CSystem&system,char const*name):CNamedComp(system,name){}
public:
	virtual void addEquation(char const*name,CNamedComp&comp,void(CNamedComp::*equation)(IEquation&equ))=0;
	/**Ajoute une �quation*/
	template<typename Type>void addEquation(char const*name,Type&comp,void(Type::*equation)(IEquation&equation)){addEquation(name,comp,static_cast<void(CNamedComp::*)(IEquation&equation)>(equation));}
	virtual void addPyEquation(char const*name,CNamedComp&comp,PyObject*pVoid)=0;//Composant n�cessaire lors de la destruction d'un composant
	void addPyEquation(char const*name,CNamedComp&comp){addPyEquation(name,comp,NULL);}
	/**Ajoute une variable � calculer par le syst�me d'�quations.*/
	virtual void addVariable(IVariable&var)=0;
	/**Demande la r�solution du syst�me (retourne VRAI si la r�solution s'est bien pass�e)*/
	virtual bool solve()=0;
	/**Demande l'impression du probl�me*/
	virtual void print(char const*fileName)const=0;
};

class PYC_PUBLIC IInequation{
protected:
	virtual~IInequation(){}
public:
	/**Ajoute un coefficient associ� � une variable*/
	virtual void setCoefficient(IVariable*var,double cR)=0;
	/**D�finit le min*/
	virtual void setMin(double Min)=0;
	/**D�finit le max*/
	virtual void setMax(double Max)=0;
	/**D�finit les limites min et max*/
	virtual void setLimits(double Max,double Min){setMin(Min),setMax(Max);}
};

struct TMILPMsgLevel{
	typedef enum EMILPMsgLevel{
		off=0,
		error,
		on,
		all
	}EMILPMsgLevel;
};
struct TMILPScalingType{
	typedef enum EMILPScalingType{
		geom_mean=1,
		equilib=16,
		near_p2=32,
		none=64,
		automatic=128
	}EMILPScalingType;
};
struct TMILPAlgorithmType{
	typedef enum EMILPAlgorithmType{
		simplex=0,
		interior_point,
		milp
	}EMILPAlgorithmType;
};
struct TMILPOptionsType{
	typedef enum EMILPOptionsType{
		none=0,
		pr_steepest_edge=1,
		hr_ratio_test=2,
		presolve=4,
		binarize=8,
		sr_heuristic=16,
		fp_heuristic=32,
		ps_heuristic=64,
		gmi_cuts=128,
		mir_cuts=256,
		cov_cuts=512,
		clq_cuts=1024
	}EMILPOptionsType;
};
struct TMILPSimplexType{
	typedef enum EMILPSimplexType{
		primal=0,
		dual,
		dual_primal
	}EMILPSimplexType;
};
struct TMILPIntPointType{
	typedef enum EMILPIntPointType{
		none=0,
		quot_min_degree,
		appr_min_degree,
		sym_appr_min_degree
	}EMILPIntPointType;
};
struct TMILPBranchingType{
	typedef enum EMILPBranchingType{
		first_frac_variable=0,
		last_frac_variable,
		most_frac_variable,
		DrTom_heuristic,
		pseudo_cost_heuristic
	}EMILPBranchingType;
};
struct TMILPBacktrackingType{
	typedef enum EMILPBacktrackingType{
		depth_first=0,
		breadth_first,
		best_local_bound,
		best_projection
	}EMILPBacktrackingType;
};
struct TMILPPreprocessingType{
	typedef enum EMILPPreprocessingType{
		none=0,
		root,
		all
	}EMILPPreprocessingType;
};
class PYC_PUBLIC IMILPManager:public CNamedComp{
protected:
	bool m_bMaximize;//Sens d'optimisation
	TMILPMsgLevel::EMILPMsgLevel m_MsgLevel;
	int m_MsgPeriod,m_MsgDelay,m_Scaling,m_TimeLimit,m_IterLimit;
	TMILPAlgorithmType::EMILPAlgorithmType m_Algo;
	int m_Options;
	TMILPSimplexType::EMILPSimplexType m_Simplex;
	TMILPIntPointType::EMILPIntPointType m_IntPoint;
	TMILPBranchingType::EMILPBranchingType m_Branching;
	TMILPBacktrackingType::EMILPBacktrackingType m_Backtracking;
	TMILPPreprocessingType::EMILPPreprocessingType m_Preprocessing;
	double m_ObjectiveVal;
	virtual void addIneq(char const*name,CNamedComp&comp,void(CNamedComp::*inequation)(IInequation&equ))=0;
	IMILPManager(CSystem&system,char const*name):CNamedComp(system,name),
		m_bMaximize(false),m_MsgLevel(TMILPMsgLevel::error),m_MsgPeriod(500),m_MsgDelay(0),m_Scaling(TMILPScalingType::none),m_TimeLimit(INT_MAX),m_IterLimit(INT_MAX),m_Algo(TMILPAlgorithmType::milp),m_Options(TMILPOptionsType::pr_steepest_edge|TMILPOptionsType::hr_ratio_test|TMILPOptionsType::presolve),m_Simplex(TMILPSimplexType::primal),m_IntPoint(TMILPIntPointType::appr_min_degree),m_Branching(TMILPBranchingType::DrTom_heuristic),m_Backtracking(TMILPBacktrackingType::best_local_bound),m_Preprocessing(TMILPPreprocessingType::all),m_ObjectiveVal(0){}
public:
	/**Ajoute une in�quation*/
	template<typename Type>void addInequation(char const*name,Type&comp,void(Type::*inequation)(IInequation&inequation)){addIneq(name,comp,static_cast<void(CNamedComp::*)(IInequation&inequation)>(inequation));}
	virtual void addPyInequation(char const*name,CNamedComp&comp,PyObject*pVoid)=0;//Composant n�cessaire lors de la destruction d'un composant
	void addPyInequation(char const*name,CNamedComp&comp){addPyInequation(name,comp,NULL);}
	virtual void removeInequation(CNamedComp&comp,char const*name)=0;
	/**Ajoute une variable � calculer par le syst�me d'�quations et son coefficient de fonction objectif.*/
	virtual void addVariable(IVariable&var,double co)=0;
	void addVariable(IVariable&var){addVariable(var,0);}
	/**Supprime la variable du syst�me*/
	virtual void removeVariable(IVariable&var)=0;
	/**D�finit un coefficient de la fonction objectif*/
	virtual void setVariableObjCoef(IVariable&var,double co)=0;
	/**D�finit les bornes min et max de la variable*/
	virtual void setVariableLimits(IVariable&var,double min,double max)=0;
	/**D�finit la borne max de la variable*/
	virtual void setVariableMax(IVariable&var,double max)=0;
	/**D�finit la borne min de la variable*/
	virtual void setVariableMin(IVariable&var,double min)=0;
	/**D�signe la variable de stockage de la valeur de la fonction objectif*/
	virtual void setObjectiveVar(IVariable*var)=0;
	/**D�finit le sens de l'optimisation*/
	void setMaximize(bool bMax){m_bMaximize=bMax;};
	bool isMaximize()const{return m_bMaximize;}
	/**D�finit l'algorithme utilis� parmi alg_simplex, alg_int_point, alg_milp*/
	void setAlgorithm(TMILPAlgorithmType::EMILPAlgorithmType algo){m_Algo=algo;}
	TMILPAlgorithmType::EMILPAlgorithmType algorithm()const{return m_Algo;}
	void setMsgOptions(TMILPMsgLevel::EMILPMsgLevel level,int period,int delay){m_MsgLevel=level;m_MsgPeriod=period;m_MsgDelay=delay;}
	TMILPMsgLevel::EMILPMsgLevel msgLevel()const{return m_MsgLevel;}
	int msgDelay()const{return m_MsgDelay;}
	int msgPeriod()const{return m_MsgPeriod;}
	void setScaling(int sc){m_Scaling=sc;}
	int scaling()const{return m_Scaling;}
	void setOptions(int opts){m_Options=opts;}
	int options()const{return m_Options;}
	void setOption(TMILPOptionsType::EMILPOptionsType opt,bool bSet){if(bSet)m_Options=m_Options|opt;else m_Options=m_Options&~opt;}
	bool option(TMILPOptionsType::EMILPOptionsType opt)const{return (opt&m_Options)!=0;}
	void addOptions(int opts){m_Options=m_Options|opts;}
	void delOptions(int opts){m_Options=m_Options&~opts;}
	void setTimeLimit(int ms){m_TimeLimit=ms;}
	int timeLimit()const{return m_TimeLimit;}
	void setIterLimit(int its){m_IterLimit=its;}
	int iterLimit()const{return m_IterLimit;}
	void setSimplexOpt(TMILPSimplexType::EMILPSimplexType smp){m_Simplex=smp;}
	TMILPSimplexType::EMILPSimplexType simplexOpt()const{return m_Simplex;}
	void setIntPointOpt(TMILPIntPointType::EMILPIntPointType ip){m_IntPoint=ip;}
	TMILPIntPointType::EMILPIntPointType intPointOpt()const{return m_IntPoint;}
	void setBacktracking(TMILPBacktrackingType::EMILPBacktrackingType bt){m_Backtracking=bt;}
	TMILPBacktrackingType::EMILPBacktrackingType backtracking()const{return m_Backtracking;}
	void setBranching(TMILPBranchingType::EMILPBranchingType br){m_Branching=br;}
	TMILPBranchingType::EMILPBranchingType branching()const{return m_Branching;}
	void setPreprocessing(TMILPPreprocessingType::EMILPPreprocessingType pre){m_Preprocessing=pre;}
	TMILPPreprocessingType::EMILPPreprocessingType preprocessing()const{return m_Preprocessing;}
	/**Retourne la valeur de la fonction objectif obtenue*/
	double objectiveValue()const{return m_ObjectiveVal;}
	/**Demande la r�solution du syst�me (retourne VRAI si la r�solution s'est bien pass�e)*/
	virtual bool solve()=0;
	/**Demande l'impression du probl�me au format mps, glp, lp suivant l'extension du fichier*/
	virtual void print(char const*fileName)const=0;
};

class PYC_PUBLIC ISystemState{
	ISystemState(ISystemState const&);
	ISystemState const&operator=(ISystemState const&);
protected:
	ISystemState(){}
public :
	virtual void reset()const=0;//!<Replace le syst�me dans l'�tat m�moris�
	virtual void release()const=0;//!<Oublie la m�morisation de l'�tat
};

/**Interface d'enregistrement d'une BdC*/
class PYC_PUBLIC IBdC
{
	char const*m_Name;
protected:
	explicit IBdC(char const*name);
public:
	virtual~IBdC();
	/**Ajoute un composant de classe clName et de nom name dans le syst�me CSystem*/
	virtual CComponent*newComponent(char const*clName,char const*name,CSystem&system)const=0;
	/**Retourne la liste des classes disponibles*/
	virtual std::vector<std::string>classes()const{return std::vector<std::string>();}
	/**Retourne la version de la BdC*/
	virtual char const*version()const{return "";}
};

#endif
