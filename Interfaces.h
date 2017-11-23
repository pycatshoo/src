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
	Cette classe n'est dérivée que par la classe CSysteme.
	Elle permet aux éléments monitorés de connaître le moniteur et de lui signaler les changements.
*/
class PYC_PUBLIC IMonitor
{
protected:
	IMonitor(void){}
public:
	/**Demande au moniteur d'enregistrer la valeur de l'élément.*/
	virtual void doMonitor(IMonitored const&elt)=0;
	/**Demande d'ajouter ou d'enlever l'élément du monitoring.*/
	virtual void monitor(IMonitored&elt,bool bMonit)=0;
};

/**Interface de base d'un élément suivi par un moniteur.
	Cette classe est spécialisée par CTransition, CState et CVariable.
	L'interface définit localement la gestion du moniteur et de la trace.
*/
class PYC_PUBLIC IMonitored{
	friend class CSystemP;
protected:
	IMonitor*m_Monitor;//!<Moniteur de suivi de l'élément.
	int m_Trace;//!<Niveau de trace de l'élément (0 = pas de trace, 1 = trace en fin de cycle, >1 = trace à chaque affectation).
	bool m_AlwaysMonitored;//!<Si vrai, l'élément est monitoré en permanence
	int m_Id;//!<Id d'identification de l'élément monitoré (défini de manière unique (si négatif : fourni et non modifiable))
	IMonitored():m_Monitor(0),m_Trace(0),m_AlwaysMonitored(false),m_Id(0){}
	virtual~IMonitored(){}
public:
	/**Retourne l'Id de l'élément monitoré.*/
	unsigned int id()const{return (unsigned)(m_Id>0?m_Id:-m_Id);}
	/**Désigne le moniteur de suivi de l'élément (si NULL, l'élément n'est pas suivi), renseigne l'id prédéfini et modifie le niveau de monitoring.*/
	virtual void setMonitor(IMonitor*monitor,unsigned id,bool always){if(monitor)monitor->monitor(*this,true);else if(m_Monitor)m_Monitor->monitor(*this,false);m_AlwaysMonitored=(monitor && always);m_Id=monitor?-(int)id:0;}
	/**Retourne le moniteur de suivi de l'élément.*/
	virtual IMonitor*monitor()const{return m_Monitor;}
	/**Modifie le niveau de trace de l'élément (si 0 pas de trace de l'élément).*/
	virtual void setTrace(int trace){m_Trace=trace;}
	/**Retourne le niveau de trace de l'élément.*/
	virtual int trace()const{return m_Trace;}
	/**Modifie l'option de monitoring permanent*/
	void setAlwaysMonitored(bool bAlways){m_AlwaysMonitored=bAlways;}
	/**Retourne vrai si l'élément est monitoré en permanence*/
	int isAlwaysMonitored()const{return m_AlwaysMonitored;}
	/**Retourne le nom simple de l'élément.*/
	virtual char const*basename()const{return 0;}
	/**Retourne le nom complet de l'élément (incluant la hiérarchie des composants propriétaires de l'élément).*/
	virtual char const*name()const=0;
	/**Retourne le type d'élément*/
	virtual char const*type()const=0;
	/**Retourne la valeur initiale float de l'élément*/
	virtual float fInitValue()const=0;
	/**Retourne l'objet parent de l'élément (dans Pycatshoo, il s'agit d'un CComponent).*/
	virtual CNamedComp*parent()const=0;
	int compare(IMonitored const&other)const;//!<Comparaison en fonction du type et du nom
	bool operator<(IMonitored const&other)const;//!<Comparaison en fonction du type et du nom
};

/**Class de gestion des différentes méthodes passées en paramètres dans Pycatshoo.
La classe permet de gérer des méthodes C++ ou des méthodes Python ou encore des fonctions Python sous la forme d'PyObject.*/
class PYC_PUBLIC CMethod{
protected:
	CNamedComp*m_Comp;
	char*m_Basename;
	bool m_Py;
	static bool st_bUseNameComp;//!<Si vrai, la comparaison des méthodes prend en compte le nom des composants, sinon, c'est uniquement l'adresse qui compte
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
	double dValue()const;//!<Retourne la valeur double obtenue par appel de dblFct(void) ou d'une fonction Python équivalente
	virtual float fValue()const;//!<Retourne la valeur float obtenue par appel de fltFct(void) ou d'une fonction Python équivalente
	int iValue()const;//!<Retourne la valeur int obtenue par appel de intFct(void) ou d'une fonction Python équivalente
	virtual bool bValue()const;//!<Retourne la valeur bool obtenue par appel de boolFct(void) ou d'une fonction Python équivalente
	double dValue(double val)const;//!<Retourne la valeur double obtenue par appel de dblFctDbl(double) ou d'une fonction Python équivalente
	void call()const;//!<Appelle la fonction voidFct(void) ou la fonction Python équivalente
	void call(char const*)const;//!<Appelle la fonction endFct(void) ou la fonction Python équivalente
	void call(IEquation&)const;//!<Appelle la fonction equation(IEquation) ou la fonction Python équivalente
	void call(IInequation&)const;//!<Appelle la fonction inequation(IInequation) ou la fonction Python équivalente
	static void setUseNameComp(bool bUseCompName);
};

struct SSensitiveMethod:CMethod{
	char m_Sens;
	SSensitiveMethod():m_Sens(0){}
	SSensitiveMethod(CNamedComp*comp,char const*name,void(CNamedComp::*method)(),int sens):CMethod(comp,name,method),m_Sens(sens){}
	SSensitiveMethod(CNamedComp*comp,char const*name,PyObject*pyObj,int sens):CMethod(comp,name,pyObj),m_Sens(sens){}
};

/**Interface d'accès à l'itérateur de séquence*/
class PYC_PUBLIC ISeqIter{
public:
	virtual~ISeqIter(){}
	virtual void restart()=0;//!<Replace l'itérateur en début de séquence
	virtual bool atEnd()const=0;//!<Retourne vrai si l'itérateur a atteint la fin de la séquence
	virtual bool atBegin()const=0;//!<Retourne vrai si l'itérateur est en début de la séquence
	virtual void next()=0;//!<Avance au pas d'enregistrement suivant (le temps peut ne pas changer)
	virtual void previous()=0;//!<Recule au pas d'enregistrement précédent (le temps peut ne pas changer)
	virtual STimeInd const&time()const=0;//!<Retourne l'instant du pas de temps courant
	virtual float value(IMonitored const*obj,bool*exists=NULL)const=0;//!<Retourne la valeur de l'élément obj à l'instant courant s'il existe
	virtual std::vector<IMonitored const*>monitoredElts()const=0;//!<Retourne la liste des éléments monitorés à l'instant courant
};

struct TOperator{
	typedef enum TOp{
		unknown,//!<Inconnu
		ne,//!<Différent
		eq,//!<Egal
		lt,//!<Strictement inférieur
		gt,//!<Strictement supérieur
		le,//!<Inférieur ou égal
		ge//Supérieur ou égal
	}TOp;
	static char const*opToStr(TOp op);
	static TOp strToOp(char const*str);
	static bool compare(float val1,TOp op,float val2);
	static bool compare(double val1,TOp op,double val2);
};

struct ICondFctFloat{//Interface des fonctors condition avec argument réel (ce n'est jamais un objet python)
	virtual ~ICondFctFloat(){}
	virtual bool operator()(float val)const{return true;}
};

struct CCondFctFloat:ICondFctFloat{//Fonctor condition construit à partir d'une fonction
	bool (*m_Condition)(float);
	CCondFctFloat(bool (*condition)(float)):m_Condition(condition){}
	inline bool operator()(float val)const{
		return m_Condition(val);
	}
};

class CCondCompFloat:public ICondFctFloat{//Fonctor condition construit à partir d'un opérateur de comparaison et d'une valeur
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

/**Interface d'accès à une séquence.
La séquence permet d'accéder aux valeurs des éléments monitorés à n'importe quel instant.
Pour suivre le déroulement d'une séquence il faut utiliser un itérateur obtenu avec la méthode iterator().
*/
class PYC_PUBLIC ISequence{
protected:
	virtual~ISequence(){}
public:
	/**Retourne un itérateur sur la séquence.*/
	virtual ISeqIter*iterator()const=0;
	/**Retourne le numéro de la portion de séquence*/
	virtual SEQ_ID num()const=0;
	/**Retourne la vraisemblance de la séquence.*/
	virtual double likelyhood()const=0;
	/**Modifie l'indice de conservation de la séquence.*/
	virtual void setToKeep(bool toKeep)=0;
	/**Retourne vrai si la séquence doit être conservée.*/
	virtual bool toKeep()const=0;
	/**Ajoute une prolongation à la séquence*/
	virtual void addExtend(ITransition*tr,PyTime date)=0;
	/**Retourne la valeur de l'objet obj à l'instant t.*/
	virtual float value(IMonitored const*obj,PyTime t)const=0;
	/**Calcule les valeurs extrêmes de l'élément monitoré (min et max doivent être initialisés).*/
	virtual void extremeValues(IMonitored const*obj,float&min,float&max)const=0;
	std::vector<float>extremeValues(IMonitored const*obj)const{std::vector<float>v(2,(float)HUGE_VAL);extremeValues(obj,v[0]=(float)-HUGE_VAL,v[1]);return v;}
	/**Calcule les temps de séjour entre les bornes.*/
	virtual void residenceTimes(IMonitored const*obj,PyTime it,PyTime ft,std::vector<float>const&lims,std::vector<PyTime>&resTs)const=0;
	std::vector<PyTime>residenceTimes(IMonitored const*obj,PyTime it,PyTime ft,std::vector<float>const&lims)const{std::vector<PyTime>v;residenceTimes(obj,it,ft,lims,v);return v;}
	/**Retourne le temps de séjour satisfaisant la condition.*/
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,ICondFctFloat const&condition)const=0;
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,char const*op,float value)const{
		return residenceTime(obj,it,ft,CCondCompFloat(op,value));
	}
	virtual float residenceTime(IMonitored const*obj,PyTime it,PyTime ft,bool (*condition)(float))const{
		return residenceTime(obj,it,ft,CCondFctFloat(condition));
	}
	/**Retourne la date de première réalisation de la condition*/
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
	/**Retourne la date d'arrêt de la séquence*/
	virtual PyTime endTime()const=0;
	/**Retourne la cause d'arrêt de la séquence*/
	virtual char const*endCause()const=0;
	/**Retourne l'intégrale de la valeur de l'objet obj entre les temps it et ft.*/
	virtual float integralValue(IMonitored const*obj,PyTime it,PyTime ft)const=0;
	/**Retourne dans values les valeurs de l'objet obj aux instants ts.*/
	virtual void values(IMonitored const*obj,std::vector<PyTime>const&ts,std::vector<float>&values)const=0;
	std::vector<float>values(IMonitored const*obj,std::vector<PyTime>const&ts)const{std::vector<float>v;values(obj,ts,v);return v;}
};

/**Interface des filtres de séquence (ce n'est jamais un objet python)
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
	/**Enuméré de définition du mode de mémorisation des indicateurs.
		Les éléments du type peuvent être cumulés.
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
	/**Enuméré de définition de la valeur calculée par l'indicateur.*/
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
	virtual void setComputation(TComputationType::EComputationType comp)=0;//!<Modifie la valeur calculée par l'indicateur
	virtual TComputationType::EComputationType computation()const=0;//!<Retourne la valeur calculée par l'indicateur
	virtual void setPctQuantileLeValue(float fracMin)=0;//!<Modifie le pourcentage de calcul du fractile min
	virtual void setPctQuantileGtValue(float fracMax)=0;//!<Modifie le pourcentage de calcul du fractile max
	virtual float pctQuantileLeValue()const=0;//!<Retourne le pourcentage de calcul du fractile min
	virtual float pctQuantileGtValue()const=0;//!<Retourne le pourcentage de calcul du fractile max
	virtual void setDistribLimits(std::vector<float>lims)=0;//!<Modifie les limites de calcul de la distribution
	virtual std::vector<float>distribLimits()const=0;//!<Retourne les limites de calcul de la distribution
	virtual std::vector<float>const&values(int i)const=0;//!<Retourne toutes les ième valeurs
	virtual float mean(int i)const=0;//!<Retourne la ième moyenne
	virtual float stdDev(int i)const=0;//!<Retourne le ième écart type
	virtual float quantileLe(int i)const=0;//!<Retourne le ième fractile min
	virtual float quantileGt(int i)const=0;//!<Retourne le ième fractile max
	virtual float confInt(float pct,int i)const=0;//!<Retourne le ième intervalle de confiance à pct%
	virtual std::vector<float>distribution(int i)const=0;//!<Retourne les probabilités de la ième distribution
	virtual std::vector<float>meanDistribution()const=0;//!<Retourne les probabilités de la distribution moyennée sur tous les instants
	virtual std::vector<float>means()const=0;//!<retourne les moyennes aux différents instants
	virtual std::vector<float>stdDevs()const=0;//!<retourne les écarts types aux différents instants
	virtual std::vector<float>quantilesLe()const=0;//!<retourne les fractiles min aux différents instants
	virtual std::vector<float>quantilesGt()const=0;//!<retourne les fractiles min aux différents instants
	virtual std::vector<float>confInts(float pct)const=0;//!<retourne les intervalles de confiance à pct% aux différents instants
	virtual void setToKeep(bool toKeep)=0;//!<Modifie l'indice de conservation de l'indicateur
	virtual bool toKeep()const=0;//!<Retourne vrai si l'indicateur doit être conservé
};

/**Interface commune à l'ensemble des variables et des références.
*/
class PYC_PUBLIC IVarBase:public CNamedComp{
	friend class CSystemState;
	virtual void addSensitiveMethod(SSensitiveMethod const&method)=0;
	CNamedComp*p()const;//!<Juste pour accéder au parent pour construire la sensitive method
protected:
	CNX_ID m_N;//!<Nombre maximum de liens acceptés par la variable.
	IVarBase(char const*name,CNamedComp*parent):CNamedComp(name,parent),m_N(0xffffffff){}
public:
	/**Retourne le type de la valeur courante de la variable.*/
	virtual std::type_info const&typeInfo()const{return typeid(bool);}
	/**Retourne le nombre maximum de connexion sur la variable.*/
	virtual CNX_ID cnctMax()const{return m_N;}
	/**Ajoute une méthode sensible aux modifications de la variable.
	@param name nom de la méthode
	@param method méthode sensible à ajouter
	@param sens sens de sensibilité de la méthode (0 tous, 1 accroissement, -1 réduction)
	*/
	template<typename Type>void addSensitiveMethod(char const*name,void(Type::*method)(),int sens=0){addSensitiveMethod(SSensitiveMethod(p(),name,static_cast<void(CNamedComp::*)()>(method),sens));}
	void addPySensitiveMethod(char const*name,PyObject*pVoid,int sens){addSensitiveMethod(SSensitiveMethod(p(),name,pVoid,sens));}
	/**Supprime la méthode de nom name des méthodes sensible aux modifications de la variable*/
	virtual void removeSensitiveMethod(char const*name)=0;
	virtual void clearSensitiveMethod()=0;
	/**Modifie le nombre maximum de connexion sur la variable*/
	void setCnctMax(CNX_ID N){m_N=N;}
};

/**Interface de gestion des variables.
	Les variables peuvent être des variables d'état ou des automates ou des états.
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
	/**Modifie la réinitialisation de la variable*/
	virtual void setReinitialized(bool toReinit){ILogManager::glLogManager().throwError("Méthode inutilisable sur cette classe");}
	/**Retourne la réinitialisation de la variable*/
	virtual bool isReinitialized()const{return false;}
	/**Modifie la gestion de l'interpolation des valeurs de la variable*/
	virtual void setInterpolated(bool interp);
	/**Retourne vrai si les valeurs de la variable doivent être interpolées*/
	virtual bool isInterpolated()const{return false;}
	/**Modifie la dérivée de la valeur pour les ODE*/
	virtual void setDvdtODE(double dvdt)=0;
	/**Modifie le fait que la variable soit modifiable de l'extérieur*/
	virtual void setModifiable(bool res)=0;
	/**Retourne la dérivée de la valeur de la variable (uniquement pour les variables double)*/
	virtual double derivative(double dt)const=0;
	/**Retourne le pas de calcul de la dérivée*/
	virtual double dtDerivative(double dt)const{return derivative(dt);/*Pour obtenir le message d'erreur par défaut*/}
	/**Désigne la variable dont la variable courante est l'intégrale.*/
	virtual void integrate(IVariable*var);
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de complexe.*/
	virtual void setValue(std::complex<double>val)=0;
	inline void setCValue(std::complex<double>val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de double.*/
	virtual void setValue(double val)=0;
	inline void setDValue(double val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de float.*/
	virtual void setValue(float val)=0;
	inline void setFValue(float val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de int.*/
	virtual void setValue(int val)=0;
	inline void setIValue(int val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de bool.*/
	virtual void setValue(bool val)=0;
	inline void setBValue(bool val){setValue(val);}
	/**Modifie la valeur courante (ou la valeur initiale si la simulation n'est pas lancée) sous forme de string.*/
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
	/**Retourne 0 si la valeur courante est indentique à la valeur initiale et + ou -1 suivant le sens de variation.*/
	virtual int valueChanged()const=0;
	/**Applique la valeur de redémarrage à la valeur courante.*/
	virtual void reset()=0;
	/**Demande la gestion du monitoring de la variable.*/
	virtual void monitorValue()const=0;
};

/**Interface de gestion des références à des variables.
	Les références sont le reflet de variables situées dans d'autres composants.
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
	/**Retourne le type de la ième connexion*/
	virtual std::type_info const&typeInfo(CNX_ID i)const=0;
	/**Retourne le nombre de valeurs connues par la référence (connexions réelles ou valeurs fixes).*/
	virtual CNX_ID cnctCount()const=0;
	virtual CNX_ID nbCnx()const{return cnctCount();}
	/**Retourne la valeur courante de la ième connexion sous forme de complexe.*/
	virtual std::complex<double>cValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la ième connexion sous forme de double.*/
	virtual double dValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la ième connexion sous forme de float.*/
	virtual float fValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la ième connexion sous forme de int.*/
	virtual int iValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la ième connexion sous forme de bool.*/
	virtual bool bValue(CNX_ID i)const=0;
	/**Retourne la valeur courante de la ième connexion castée au type T.*/
	template<typename T>T value(CNX_ID i)const{
		T res;
		value(res,i);
		return res;
	}
	/**Retourne la valeur courante de la ième connexion sous forme de string.*/
	virtual std::string sValue(CNX_ID i)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de complexe.*/
	virtual CNX_ID cValues(std::complex<double>*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de double.*/
	virtual CNX_ID dValues(double*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de float.*/
	virtual CNX_ID fValues(float*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme d'entiers.*/
	virtual CNX_ID iValues(int*tab)const=0;
	/**Retourne les valeurs courantes des connexions sous forme de booléens.*/
	virtual CNX_ID bValues(bool*tab)const=0;
	/**Retourne la référence associée à la ième connexion.*/
	virtual IReference*reference(CNX_ID i)const=0;
	/**Retourne la variable associée à la ième connexion.*/
	virtual IVariable*variable(CNX_ID i)const=0;
	/**Retourne la IVarBase associée à la ième connexion.*/
	virtual IVarBase*varBase(CNX_ID i)const=0;
	/**Prend une quantité res de ressource à la ième variable connectée.*/
	virtual void takeResource(CNX_ID i,double res)=0;
	/**Restitue une quantité res de ressource à la ième variable connectée.*/
	virtual void restoreResource(CNX_ID i,double res)=0;
	/**Modifie la valeur de la ième variable connectée.*/
	virtual void setValue(CNX_ID i,double val)=0;
	/**Modifie la dérivée de la ième variable connectée (uniquement si c'est une variable double).*/
	virtual void setDvdt(CNX_ID i,double val)=0;
	/**Lie la référence à une autre référence ou variable vb*/
	virtual void bind(IVarBase&vb)=0;
	/**Délie la référence de la référence ou de la variable vb*/
	virtual void unbind(IVarBase&vb)=0;
	/**Délie la ième connexion*/
	virtual void unbind(CNX_ID i)=0;
	/**Calcule un ou sur l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual bool orValue(bool def=false)const=0;
	/**Calcule un et sur l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual bool andValue(bool def=true)const=0;
	/**Calcule une somme de l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual double sumValue(double def=0)const=0;
	/**Calcule un produit l'ensemble des connexions (def est la valeur en l'absence de connexion)*/
	virtual double productValue(double def=1)const=0;
	/**Remplit le vecteur avec les indices qui vérifient la condition*/
	virtual int valuesThat(ICondFctFloat const&cf, std::vector<int> & indexes)const=0;
	virtual int valuesThat(bool(condition)(float), std::vector<int> & indexes)const{return valuesThat(CCondFctFloat(condition),indexes);}
};

/**Interface de gestion des boîtes de messages.
*/
class PYC_PUBLIC IMessageBox:public CNamedComp{
protected:
	IMessageBox(CNamedComp&parent,char const*name):CNamedComp(name,&parent){}
public:
	/**Ajoute une variable à connecter.*/
	virtual void addExport(IVarBase*p,char const*aliasName=NULL)=0;
	/**Retourne la variable exportée sous l'alias aliasName (exception si c'est une référence).*/
	virtual IVariable*exportedVariable(char const*aliasName)const=0;
	/**Ajoute une référence à connecter et définit le nom de la variable à lui associer.*/
	virtual void addImport(IReference*p,char const*varName=NULL)=0;
	/**Ajoute une référence à connecter facultativement et définit le nom de la variable à lui associer et sa valeur par défaut s'il n'est pas connecté.*/
	virtual void addOptionalImport(IReference*p,int type, double defaultValue, char const*alias)=0;
	//*Retourne vrai si la connection peut être effectuée*/
	virtual bool canConnectTo(IMessageBox&other)const=0;
	//*Retourne vrai si la connection existe déjà*/
	virtual bool isConnectedTo(IMessageBox&other)const=0;
	/**Connecte la boîte de messages à une autre boîte de messages.*/
	virtual void connectTo(IMessageBox&other,double weight)=0;
	void connectTo(IMessageBox&other){connectTo(other,0);}//Pour le python
	/**deconnecte la boîte de messages d'une autre boîte de messages.*/
	virtual void disconnectFrom(IMessageBox&other)=0;
	/**deconnecte la boîte de messages des autres boîtes de messages.*/
	virtual void disconnect()=0;
	/**Retourne le nombre de connexions*/
	virtual CNX_ID cnctCount()const=0;
	/**Retourne le poids de la ième connection*/
	virtual double weight(CNX_ID i)const=0;
	/**Retourne le ième composant connecté*/
	virtual CComponent*component(CNX_ID i)const=0;
	/**Retourne la ième boîte de message connectée*/
	virtual IMessageBox*cnct(CNX_ID i)const=0;
	/**Retourne sous forme de complexe la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual std::complex<double> cValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de double la valeur la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual double dValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de float la valeur la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual float fValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de int la valeur la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual int iValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de bool la valeur la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual bool bValue(CNX_ID i,char const*aliasName)const=0;
	/**Retourne sous forme de string la valeur la valeur de la ième variable connectée exportée sous l'alias aliasName.*/
	virtual std::string sValue(CNX_ID i,char const*aliasName)const=0;
};

struct SBindingMM;
struct SBindingM;
struct SBinding;
/**Interface des foncteurs de modification de loi de probabilité
Le foncteur modifie la valeur "normale" qui est fournie en iniVal.
*/
struct IDistLawModifier{
	virtual double operator()(double iniVal)const=0;
	~IDistLawModifier(){}
};
struct TLawType{
	/**Définit les types de loi prédéfinis dans le coeur de Pycatshoo*/
	typedef enum ELawType{
		inst,//!<Loi instantanée
		expo,//!<Loi exponentielle
		cstt,//!<Loi temps constant
		defer=cstt,
		cstd//!<Loi date constante
	}ELawType;
};
/**Interface de gestion des lois de probabilité.
*/
class PYC_PUBLIC IDistLaw{
	friend class CDistLawInst;
	friend class ITransition;
	friend class CTransition;
	LAWP_ID m_NbParam;
	CNamedComp&m_Parent;
	SBindingMM*m_Parameters;//!<Définition première des paramètres permettant de mettre à jour les valeurs numériques
	double*m_ParamValues;//!<Valeurs des paramètres utilisées pour le calcul du temps d'attente
	void setParameter(double(CNamedComp::*fct)(),LAWP_ID place);
	void reset();//!<Replace les valeurs initiales
	void insertAt(LAWP_ID place,bool bAdd=false);//!<Crée une place en place
	void incRef();
	unsigned int m_NbCppRef;//!<Nombre de références d'un objet C++
	PyObject*m_Self;//!<Référence à l'objet python correspondant (NULL, si c'est un objet purement C++).
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
	/**Définit le nombre de paramètres de la loi (par défaut 1)*/
	void setNbParameters(LAWP_ID nb){if(nb>m_NbParam)insertAt(nb-1);else m_NbParam=nb;}
	/**Insert un paramètre sous forme de valeur.*/
	void insertParameter(double value,LAWP_ID place=0);
	/**Insert un paramètre sous forme de variable.*/
	void insertParameter(IVariable&var,LAWP_ID place=0);
	/**Définit un paramètre par connexion d'une variable.*/
	void setParameter(IVariable&var,LAWP_ID place=0);
	/**Définit un paramètre par une méthode*/
	template<typename Type>void setParameter(double (Type::*fct)(),LAWP_ID place=0){setParameter(static_cast<double (CNamedComp::*)()>(fct),place);}
	/**Modifie la valeur d'un paramètre.*/
	void setParameter(double value,LAWP_ID place=0);
	/**Définit un paramètre par une fonction Python*/
	void setPyParameter(char const*name,PyObject*pyObj,LAWP_ID place);
	/**Définit un modificateur pour un paramètre existant*/
	void setParameterModifier(IDistLawModifier const*dlModif,LAWP_ID place);
	/**Retourne la valeur d'un paramètre*/
	virtual double parameter(LAWP_ID place=0)const{return m_ParamValues[place];}
	/**Retourne le temps d'attente tiré aléatoirement avec les paramètres courants de telle sorte que le délai soit supérieur à dMin*/
	virtual PyTime delayBeyond(PyTime dMin)const{return dMin;}
	/**Retourne le temps d'attente tiré aléatoirement avec les paramètres courants*/
	virtual PyTime delay()const{return delayBeyond(0.);}
	/**Retourne le temps minimal d'attente*/
	virtual PyTime delayMin()const{return 0.;}
	/**Retourne le temps minimal d'attente*/
	virtual PyTime delayMax()const{return HUGE_VAL;}
	/**Retourne la valeur de la densité de probabilité*/
	virtual double density(PyTime t)const{return 0.;}
	/**Retourne la valeur de la fonction de répartition*/
	virtual double distribution(PyTime t)const{return 0.;}
	/**Retourne l'indexe de la transition de sortie tiré aléatoirement avec les paramètres courants (à partir de 0 jusqu'à nbIndex-1)*/
	virtual TGT_ID index()const{return 0;}
	/**Retourne le nombre de paramètres de la loi*/
	inline LAWP_ID nbParam()const{return m_NbParam;}
	/**Retourne le nombre de sorties gérées par la loi.*/
	virtual TGT_ID nbIndex()const{return 1;}
	/**Retourne faux si les paramètres ont changé*/
	bool upToDate();
	/**Construit une des lois prédéfinies*/
	static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,IVariable&var);
	/**Construit une des lois prédéfinies*/
	static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,double param);
	/**Construit une des lois prédéfinies*/
	template<typename Type>static IDistLaw*newLaw(CNamedComp&parent,TLawType::ELawType type,double (Type::*fct)()){IDistLaw*dl=newLaw(parent,type,0);dl->setParameter(fct);return dl;}
	static IDistLaw*newPyLaw(CNamedComp&parent,TLawType::ELawType type,char const*name,PyObject*pVoid){IDistLaw*dl=newLaw(parent,type,0);dl->setPyParameter(name,pVoid,0);return dl;}
	static IDistLaw*newPyLaw(CNamedComp&parent,TLawType::ELawType type,char const*name){return newPyLaw(parent,type,name,NULL);}
};

struct TModificationMode{
	/**Enuméré de définition de l'aspect modifiable de la transition.*/
	typedef enum EModificationMode{
		not_modifiable,//!<Transition non modifiable
		not_modif=not_modifiable,
		discrete_modification,//!<Transition modifiable à chaque pas
		modif=discrete_modification,
		continuous_modification,//!<Transition modifiable continûment (dans les ODE)
		cont_modif=continuous_modification
	}EModificationMode;
};
struct TTransType{
	/**Enuméré de définition de type de franchissement de la transition.*/
	typedef enum ETransType{
		trans=1,//!<Transition standard
		fault=2,//!<Défaillance
		rep=3//!<Réparation
	}ETransType;
	static char const*st_Names[];
};
/**Interface de gestion des transitions d'un état à un autre.
	Une transition permet de contrôler le passage d'un état à un autre.
	Une transition gère :
	- une condition par l'intermédiaire d'une référence m_Condition,
	- des paramètres de loi par l'intermédiaire de la référence m_Parameters.
	Une transition connaît son état de départ et ses états d'arrivée potentiels.
	Il ne peut y avoir plusieurs états d'arrivée que pour une transition INS.

	Les modèles actuellement reconnus sont :
	- INS : les paramètres sont les proba de passage dans les états cibles (le dernier paramètre peut être ignoré)
	- EXP : l'unique paramètre est le taux de passage par unité de temps (seconde)
	- TC  : l'unique paramètre est la durée d'attente en seconde

	La transition peut être qualifée de :
	- trans : transition standard,
	- def : défaillance,
	- rep : réparation.

	La transition peut être :
	- interruptible (si la condition est invalidée pendant l'attente, l'attente est interrompue),
	- modifiable (si les paramètres sont modifiés pendant l'attente, celle-ci est recalculée).
*/
class PYC_PUBLIC ITransition : public CNamedComp,public IMonitored{
protected:
	SBindingM&m_Condition;//!<Définition de la condition de franchissement de la transition
	bool m_NCond;//!<Si vrai, la condition est utilisée à l'envers
	bool m_NCondInit;//!<Si vrai, la condition est initialement utilisée à l'envers
	bool m_Interruptible;//!<Si vrai, la transition peut être interrompue si la condition n'est plus valide (par défaut faux)
	TModificationMode::EModificationMode m_Modifiable;//!<Si 1 la transition peut être modifiée après son lancement, si 2 elle est continuement modifiable (intégration) (par défaut non modifiable)
	IDistLaw*m_DistLaw;//!<Loi de distribution des probabilités
	virtual void addSensitiveMethod(SSensitiveMethod const&method)=0;
	void setCondition(bool(CNamedComp::*fct)(),bool negate);
	ITransition(CNamedComp&parent,char const*name);
	virtual~ITransition();
public:
	char const*basename()const override{return CNamedComp::basename();}
	char const*name()const override{return CNamedComp::name();}
	CNamedComp*parent()const override{return CNamedComp::parent();}
	/**Retourne la loi de probabilité de la transition*/
	IDistLaw*distLaw()const{return m_DistLaw;}
	/**Modifie la loi de probabilité de la transition*/
	void setDistLaw(IDistLaw*distLaw);
	/**Modifie la loi de probabilité de la transition*/
	IDistLaw*setDistLaw(TLawType::ELawType type,double param){setDistLaw(IDistLaw::newLaw(*parent(),type,param));return distLaw();}
	IDistLaw*setDistLaw(TLawType::ELawType type,IVariable&param){setDistLaw(IDistLaw::newLaw(*parent(),type,param));return distLaw();}
	template<typename Type>IDistLaw*setDistLaw(TLawType::ELawType type,double(Type::*fct)()){setDistLaw(IDistLaw::newLaw(*parent(),type,fct));return distLaw();}
	IDistLaw*setPyDistLaw(TLawType::ELawType type,char const*name,PyObject*pVoid){setDistLaw(IDistLaw::newPyLaw(*parent(),type,name,pVoid));return distLaw();}
	IDistLaw*setPyDistLaw(TLawType::ELawType type,char const*name){setPyDistLaw(type,name,NULL);return distLaw();}
	/**Modifie le caractère interruptible de la transition (par défaut faux).*/
	void setInterruptible(bool interruptible){m_Interruptible=interruptible;}
	/**Retourne le cratère interruptible de la transition.*/
	bool interruptible()const{return m_Interruptible;}
	/**Modifie le caractère modifiable de la transition (par défaut faux).*/
	void setModifiable(TModificationMode::EModificationMode modifiable){m_Modifiable=modifiable;}
	/**Retourne le cratère modifiable de la transition.*/
	TModificationMode::EModificationMode modifiable()const{return m_Modifiable;}
	/**Retourne l'état de départ de la transition*/
	virtual IState const*startState()const=0;
	/**Retourne le nombre de cibles de la transition*/
	virtual TGT_ID targetCount()const=0;
	/**Insert un état cible à la transition et définit le type de la transition vers cet état.*/
	virtual void insertTarget(IState*end,int type,int place)=0;
	/**Ajoute un état cible à la transition et définit le type de la transition vers cet état.*/
	virtual void addTarget(IState*end,int type=TTransType::trans)=0;
	/**Retourne l'état cible d'indice i.*/
	virtual IState const*getTarget(TGT_ID i)const=0;
	/**Retourne le type de franchissement effectué pour atteindre le ième état cible.*/
	virtual int getTargetType(TGT_ID ind)const=0;
	/**Inhibe ou désinhibe les sorties vers l'état*/
	virtual void inhibateTarget(IState*st,bool inhibate)=0;
	/**Inhibe ou désinhibe la sortie vers les cibles de type type*/
	virtual void inhibateTarget(int type,bool inhibate)=0;
	/**Renseigne la condition par connexion à une variable ou à un état.*/
	void setCondition(IVariable&var,bool negate=false);
	void setCondition(IVariable*var,bool negate=false){setCondition(*var,negate);}
	/**Renseigne la condition par une fonction booléenne.*/
	template<typename FctType> void setCondition(FctType fct,bool negate=false){setCondition(static_cast<bool(CNamedComp::*)()>(fct),negate);}
	/**Renseigne la condition par une fonction Python.*/
	void setPyCondition(char const*name,PyObject*pyObj,bool negate=false);
	/**Renseigne la condition par une méthode Python nommée.*/
	void setPyCondition(char const*name,bool negate=false){setPyCondition(name,NULL,negate);}
	/**Renseigne la condition par une valeur booléenne.*/
	void setCondition(bool val);
	/**Retourne la valeur actuelle de la condition.*/
	bool getCondition()const;
	/**Retourne la date de franchissement prévue (ou -1 s'il n'y a pas de date prévue).*/
	virtual PyTime endTime()const=0;
	/**Retourne la date de décision d'activation*/
	virtual PyTime startTime()const=0;
	/**Modifie le délai d'attente*/
	virtual void setDelay(PyTime delay)=0;
	/**Retourne l'indice de l'état de sortie choisi ou -1 sinon*/
	virtual int indOutState()const=0;
	/**Modifie l'indice de l'état de sortie*/
	virtual void setIndOutState(unsigned ind)=0;
	/**Ajoute une méthode sensible au tirage de la transition*/
	template<typename FctType>void addSensitiveMethod(char const*name,FctType method,int outState=-1){addSensitiveMethod(SSensitiveMethod(parent(),name,static_cast<void(CNamedComp::*)()>(method),outState));}
	void addPySensitiveMethod(char const*name,PyObject*pVoid,int outState=-1);
	/**Supprime une méthode sensible au tirage de la transition*/
	virtual void removeSensitiveMethod(char const*name)=0;
	/**Retourne l'indice de l'état de sortie atteint lors du tirage de la transition (ou -1 si la transition n'a pas été tirée)*/
	virtual int firedState()const=0;
	/**Force à recalculer le délai d'attente*/
	virtual void invalidate()=0;
	/**Modifie le masque des états de sortie à monitorer*/
	virtual void setMonitoredOutStateMask(char const*mask)=0;
	/**Retourne le masque des états de sortie à monitorer*/
	virtual char const*monitoredOutStateMask()const=0;
};

/**Interface de gestion des états.
	Les états permettent de construire le diagramme d'états d'un composant à l'aide des transitions.
	Un état peut être actif ou non. Un état actif active ses transitions de sortie.
	Un état peut être exporté dans une boîte de messages comme une vzariable.
*/
class PYC_PUBLIC IState:public virtual IVariable{
public:
	/**Ajoute une transition de sortie de l'état.*/
	virtual ITransition*addTransition(char const*name)=0;
	/**Retourne vrai si l'état est actif.*/
	virtual bool isActive()const{return bValue();}
	/**Retourne la date de sortie de l'état la plus proche (en seconde).*/
	virtual double exitTime()const=0;
	/**Retourne l'automate de l'état.*/
	virtual IAutomaton*automaton()const=0;
	/**Retourne l'indice de l'état (doit être unique dans l'automate).*/
	virtual unsigned int index()const=0;
	/**Retourne la liste des transitions au départ de l'état.*/
	virtual std::vector<ITransition*>transitions()const=0;
};

/**Interface de gestion des automates d'états.
	Les automates gèrent le diagramme des états d'un composant.
	Un automate peut restituer l'état courant ou sont index et être exporté dans une boîte de messages comme une variable.
*/
class PYC_PUBLIC IAutomaton:public virtual IVariable{
public:
	virtual size_t nbStates()const=0;//Retourne le nombre d'états de l'automate
	virtual std::vector<IState*>states()const=0;//!<Retourne la liste des états de l'automate
	/**Modifie l'état initial de l'automate.*/
	virtual void setInitState(IState*st)=0;
	/**Retourne l'état initial ou NULL.*/
	virtual IState*initState()const=0;
	/**Retourne l'indice de l'état courrent ou -1.*/
	virtual int currentIndex()const=0;
	/**Retourne l'état courant ou NULL.*/
	virtual IState*currentState()const=0;
	/**Ajoute un état à l'automate.*/
	virtual IState*addState(char const*name,int index)=0;
};

/**Interface des conditions d'étapes (ce n'est jamais un objet python)
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

/**Interface de manipulation des étapes.*/
struct IStep{
public:
	/**Retourne le nom de l'étape*/
	virtual char const*name()const=0;
	/**Ajoute une méthode à l'étape*/
	virtual void addMethod(CNamedComp const*cmp,char const*methodName)=0;
	/**Modifie la condition de l'étape.
	L'étape prend possession du fonctor condition*/
	virtual void setCondition(ICondFct*fct)=0;
	/**Modifie la condition de l'étape.*/
	void setCondition(bool(*fct)()){setCondition(new CCondFct(fct));}
	/**Retourne vrai si la condition de l'étape est satisfaite*/
	virtual bool condition()const=0;
};

struct TSchemaType{
	/**Enuméré de choix de l'algorithme d'intégration.
	*/
	typedef enum ESchemaType{
		euler=1,//!<Algorithme d'euler
		runge_kutta4,//!<Runge kutta d'ordre 4
		runge_kutta_cash_karp54,//!<Runge kutta d'ordre 4/5
		runge_kutta_dopri5,//!<Runge kutta d'ordre 5
		runge_kutta_fehlberg78,//!<Runge kutta fehlberg
		modified_midpoint,//!<Point milieu modifié
		controlled_rk_cash_karp54,//!<Runge kutta à pas variable
		controlled_rk_dopri5,//!<Runge kutta à pas variable
		controlled_rk_fehlberg78,//!<Runge kutta à pas variable
		controlled_bulirsch_stoer,//!<Bulirsch Stoer naturellement à pas variable
		dense_rk_dopri5,//!<Runge kutta dense et à pas variable
		dense_bulirsch_stoer//!<Bulirsch Stoer dense et à pas variable
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

	virtual void addEquationMethod(TODEFct const&fct)=0;//!<Ajoute une fonction de calcul des dérivées des variables
	virtual void addBeginMethod(CMethod const&fct)=0;//!<Ajoute une fonction à appeler avant l'intégration
	virtual void addEndMethod(CMethod const&fct)=0;//!<Ajoute une fonction à appeler après l'intégration
	virtual void addCondition(CMethod const&fct)=0;//!<Ajoute une fonction de calcul de condition d'arrêt de l'intégration
	virtual void addCondition(CTransition*tr)=0;//!<Ajoute une transition utilisée pour sa condition
	int m_Trace;
public:
	/**Modifie l'algorithme d'intégration (par défaut Runge kutta d'ordre 4).*/
	virtual void setSchema(TSchemaType::ESchemaType schema)=0;
	/**Retourne l'algorithme d'intégration utilisé.*/
	virtual TSchemaType::ESchemaType schema()const=0;
	/**Modifie le pas d'intégration (par défaut 0.01).*/
	virtual void setDt(double dt)=0;
	virtual double getDt()const=0;
	/**Modifie le pas d'intégration minimum (par défaut 0).*/
	virtual void setDtMin(double dtMin)=0;
	virtual double getDtMin()const=0;
	/**Modifie le pas d'intégration maximum (par défaut l'infini).*/
	virtual void setDtMax(double dtMax)=0;
	virtual double getDtMax()const=0;
	/**Modifie la précision de recherche des intersections (par défaut 0.001).*/
	virtual void setDtCond(double dt)=0;
	virtual double getDtCond()const=0;
	/**Modifie la périodicité de mémorisation des variables (par défaut 0.1).*/
	virtual void setDtMem(double dt)=0;
	virtual double getDtMem()const=0;
	/**Modifie l'option de stockage systématique des variables (par défaut false).*/
	virtual void setAlwaysMem(bool bAlways)=0;
	virtual bool alwaysMem()const=0;
	/**Ajoute une variable à gérer par le système d'ODE.*/
	virtual void addODEVariable(IVariable&var)=0;
	/**Ajoute une variable à calculer en même temps que le système d'ODE.*/
	virtual void addExplicitVariable(IVariable&var)=0;
	/**Ajoute une fonction à appeler avant l'intégration*/
	template<typename Type> void addBeginMethod(char const*name,Type&comp,void(Type::*beginFct)()){addBeginMethod(CMethod(&comp,name,static_cast<void(CNamedComp::*)()>(beginFct)));}
	/**Ajoute une fonction à appeler après l'intégration*/
	template<typename Type> void addEndMethod(char const*name,Type&comp,void(Type::*endFct)(char const*)){addEndMethod(CMethod(&comp,name,static_cast<void(CNamedComp::*)(char const*)>(endFct)));}
	/**Ajoute une fonction d'équation différentielle*/
	template<typename Type> void addEquationMethod(char const*name,Type&comp,void(Type::*odeFct)(),int order=0){addEquationMethod(TODEFct(&comp,name,static_cast<void(CNamedComp::*)()>(odeFct),order));}
	/**Ajoute une fonction de calcul de condition*/
	template<typename Type> void addCondition(char const*name,Type&comp,double(Type::*condFct)()){addCondition(CMethod(&comp,name,static_cast<double(CNamedComp::*)()>(condFct)));}
	template<typename Type> void addBoundaryCheckerMethod(char const*name,Type&comp,double(Type::*condFct)()){addCondition(CMethod(&comp,name,static_cast<double(CNamedComp::*)()>(condFct)));}
	/**Ajoute une transition utilisée pour sa condition*/
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

/**Classe de définition d'une équation*/
class PYC_PUBLIC IEquation{
protected:
	virtual~IEquation(){}
public:
	/**Ajoute un coefficient associé à une variable*/
	virtual void setCoefficient(IVariable*var,double cR,double cI=0)=0;
	/**Définit la constate du second membre*/
	virtual void setConstant(double cR,double cI=0)=0;
};

/**Classe de définition et de résolution de systèmes d'équations linéaires.
*/
class PYC_PUBLIC ISLEManager:public CNamedComp{
protected:
	ISLEManager(CSystem&system,char const*name):CNamedComp(system,name){}
public:
	virtual void addEquation(char const*name,CNamedComp&comp,void(CNamedComp::*equation)(IEquation&equ))=0;
	/**Ajoute une équation*/
	template<typename Type>void addEquation(char const*name,Type&comp,void(Type::*equation)(IEquation&equation)){addEquation(name,comp,static_cast<void(CNamedComp::*)(IEquation&equation)>(equation));}
	virtual void addPyEquation(char const*name,CNamedComp&comp,PyObject*pVoid)=0;//Composant nécessaire lors de la destruction d'un composant
	void addPyEquation(char const*name,CNamedComp&comp){addPyEquation(name,comp,NULL);}
	/**Ajoute une variable à calculer par le système d'équations.*/
	virtual void addVariable(IVariable&var)=0;
	/**Demande la résolution du système (retourne VRAI si la résolution s'est bien passée)*/
	virtual bool solve()=0;
	/**Demande l'impression du problème*/
	virtual void print(char const*fileName)const=0;
};

class PYC_PUBLIC IInequation{
protected:
	virtual~IInequation(){}
public:
	/**Ajoute un coefficient associé à une variable*/
	virtual void setCoefficient(IVariable*var,double cR)=0;
	/**Définit le min*/
	virtual void setMin(double Min)=0;
	/**Définit le max*/
	virtual void setMax(double Max)=0;
	/**Définit les limites min et max*/
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
	/**Ajoute une inéquation*/
	template<typename Type>void addInequation(char const*name,Type&comp,void(Type::*inequation)(IInequation&inequation)){addIneq(name,comp,static_cast<void(CNamedComp::*)(IInequation&inequation)>(inequation));}
	virtual void addPyInequation(char const*name,CNamedComp&comp,PyObject*pVoid)=0;//Composant nécessaire lors de la destruction d'un composant
	void addPyInequation(char const*name,CNamedComp&comp){addPyInequation(name,comp,NULL);}
	virtual void removeInequation(CNamedComp&comp,char const*name)=0;
	/**Ajoute une variable à calculer par le système d'équations et son coefficient de fonction objectif.*/
	virtual void addVariable(IVariable&var,double co)=0;
	void addVariable(IVariable&var){addVariable(var,0);}
	/**Supprime la variable du système*/
	virtual void removeVariable(IVariable&var)=0;
	/**Définit un coefficient de la fonction objectif*/
	virtual void setVariableObjCoef(IVariable&var,double co)=0;
	/**Définit les bornes min et max de la variable*/
	virtual void setVariableLimits(IVariable&var,double min,double max)=0;
	/**Définit la borne max de la variable*/
	virtual void setVariableMax(IVariable&var,double max)=0;
	/**Définit la borne min de la variable*/
	virtual void setVariableMin(IVariable&var,double min)=0;
	/**Désigne la variable de stockage de la valeur de la fonction objectif*/
	virtual void setObjectiveVar(IVariable*var)=0;
	/**Définit le sens de l'optimisation*/
	void setMaximize(bool bMax){m_bMaximize=bMax;};
	bool isMaximize()const{return m_bMaximize;}
	/**Définit l'algorithme utilisé parmi alg_simplex, alg_int_point, alg_milp*/
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
	/**Demande la résolution du système (retourne VRAI si la résolution s'est bien passée)*/
	virtual bool solve()=0;
	/**Demande l'impression du problème au format mps, glp, lp suivant l'extension du fichier*/
	virtual void print(char const*fileName)const=0;
};

class PYC_PUBLIC ISystemState{
	ISystemState(ISystemState const&);
	ISystemState const&operator=(ISystemState const&);
protected:
	ISystemState(){}
public :
	virtual void reset()const=0;//!<Replace le système dans l'état mémorisé
	virtual void release()const=0;//!<Oublie la mémorisation de l'état
};

/**Interface d'enregistrement d'une BdC*/
class PYC_PUBLIC IBdC
{
	char const*m_Name;
protected:
	explicit IBdC(char const*name);
public:
	virtual~IBdC();
	/**Ajoute un composant de classe clName et de nom name dans le système CSystem*/
	virtual CComponent*newComponent(char const*clName,char const*name,CSystem&system)const=0;
	/**Retourne la liste des classes disponibles*/
	virtual std::vector<std::string>classes()const{return std::vector<std::string>();}
	/**Retourne la version de la BdC*/
	virtual char const*version()const{return "";}
};

#endif
