/*************************************
*         Copyright 2015 EDF         *
*************************************/
#include "Analyser.h"

#include "SystemP.h"
#include "Variable.h"
#include "Sequence.h"
#include "XMLTools.h"
#include "Transition.h"
#include "Execute.h"
#include "HoldAnyCvt.h"
#include "SequenceFilter.h"

#include <string.h>
#include <algorithm>
#include "trng/special_functions.hpp"

struct STrans{
	CTransition const*m_Trans;
	int m_OutInd;
	char const*m_Name;
	STrans(CTransition const*trans=NULL,int ind=0,char const*name=""):m_Trans(trans),m_OutInd(ind),m_Name(name){}
	bool operator<(STrans const&other)const{
		int cmp=strcmp(m_Name,other.m_Name);
		return cmp<0?true:cmp>0?false:(other.m_OutInd<m_OutInd);
	}
};

struct SRedBranch:public std::pair<double,std::multiset<STrans> >{
	explicit SRedBranch(double t)
		:std::pair<double,std::multiset<STrans> >(t,std::multiset<STrans>())
	{}
};

/**Classes de transitions (pour chaque transition et index de sortie, le nom de la classe)*/
class CTrClasses:public std::map<std::pair<CTransition const*,int>,std::string >{
};

/**Classes de séquences (pour chaque nom de classe, le filtre correspondant)*/
class CSeqClasses:public std::map<std::string,IFilterFct*>{
};

class CRedSeq:public std::vector<SRedBranch*>{
	SEQ_ID m_Nb,m_Id;
public:
	explicit CRedSeq(CSequence const&seq);
	CRedSeq(CRedSeq const&rs,std::map<std::pair<CTransition const*,int>,std::string >const&classes);
	CRedSeq(CRedSeq const&rs):m_Nb(rs.m_Nb),m_Id(rs.m_Id){
		reserve(rs.size());
		for(std::vector<SRedBranch*>::const_iterator it=rs.begin();it!=rs.end();it++)
			push_back(new SRedBranch(**it));
	}
	~CRedSeq(){
		for(std::vector<SRedBranch*>::iterator it=begin();it!=end();it++)
			delete*it;
	}
	CRedSeq&operator=(CRedSeq const&rs){
		clear();
		reserve(rs.size());
		m_Nb=rs.m_Nb;
		m_Id=rs.m_Id;
		for(std::vector<SRedBranch*>::const_iterator it=rs.begin();it!=rs.end();it++)
			push_back(new SRedBranch(**it));
		return*this;
	}
	void clear(){
		for(std::vector<SRedBranch*>::iterator it=begin();it!=end();it++)
			delete*it;
		std::vector<SRedBranch*>::clear();
	}
	void add(CRedSeq const&seq);
	SEQ_ID nb()const{return m_Nb;}
	void setNb(SEQ_ID nb){m_Nb=nb;}
	SEQ_ID id()const{return m_Id;}
	void setId(SEQ_ID id){m_Id = id;}
	/**Retourne une valeur positive si seq>*this, négative si seq<*this et nulle si seq=*this*/
	int compare(CRedSeq const&seq)const;
	bool operator<(CRedSeq const&seq)const;
	static bool nbCompare(CRedSeq const*rs1,CRedSeq const*rs2){return (rs1->m_Nb==rs2->m_Nb)?rs1->m_Id>rs2->m_Id:(rs1->m_Nb<rs2->m_Nb);}
	void dump(CXMLNode&nSeq,bool bId)const{
		nSeq.add_attr("N",m_Nb);
		if(bId)
			nSeq.add_attr("ID",id());
		for(std::vector<SRedBranch*>::const_iterator itb=begin();itb!=end();itb++){
			CXMLNode&nBr=nSeq.add_son("BR");
			nBr.add_attr("T",(*itb)->first/m_Nb);
			for(std::multiset<STrans>::const_iterator itt=(*itb)->second.begin();itt!=(*itb)->second.end();itt++){
				CXMLNode&nTr=nBr.add_son("TR");
				nTr.add_attr("NAME",itt->m_Name);
				if(itt->m_OutInd>=0){
					nTr.add_attr("ST",(itt->m_Trans->getTarget(itt->m_OutInd)?itt->m_Trans->getTarget(itt->m_OutInd):itt->m_Trans->startState())->basename());
					nTr.add_attr("I",itt->m_OutInd);
					nTr.add_attr("TT",itt->m_Trans->getTargetType(itt->m_OutInd));
					nTr.add_attr("TD",itt->m_Trans->distLaw()->name());
				}
			}
		}
	}
};

CRedSeq::CRedSeq(CSequence const&seq):m_Nb(1),m_Id(seq.num()){
	//On suppose que c'est une fin de séquence
	for(CSequence::CIter it(seq);!it.atEnd();it.next()){
		SRedBranch*tr=NULL;
		for(std::map<IMonitored const*,CHoldAny>::const_iterator itm=it.data().begin();itm!=it.data().end();itm++)
			if(dynamic_cast<CTransition const*>(itm->first)){
				if(!tr)
					push_back(tr=new SRedBranch(it.time().m_Time));
				tr->second.insert(STrans(dynamic_cast<CTransition const*>(itm->first),convertHA<int>(itm->second),itm->first->name()));
			}
	}
}

CRedSeq::CRedSeq(CRedSeq const&rs,std::map<std::pair<CTransition const*,int>,std::string >const&classes):m_Nb(rs.m_Nb),m_Id(rs.m_Id){
	reserve(rs.size());
	for(std::vector<SRedBranch*>::const_iterator it=rs.begin();it!=rs.end();it++){
		SRedBranch*tr=new SRedBranch((*it)->first);
		for(auto itb = (*it)->second.begin();itb!=(*it)->second.end();itb++){
			std::map<std::pair<CTransition const*,int>,std::string >::const_iterator itc=classes.find(std::pair<CTransition const*,int>(itb->m_Trans,itb->m_OutInd));
			tr->second.insert(STrans(itb->m_Trans,itc==classes.end()?itb->m_OutInd:-itc->first.second-1,itc==classes.end()?itb->m_Trans->name():itc->second.c_str()));
		}
		push_back(tr);
	}
}

int CRedSeq::compare(CRedSeq const&seq)const{
	for(std::vector<SRedBranch*>::const_iterator it1=begin(),it2=seq.begin();;it1++,it2++){
		if(it1==end()){
			if(it2==seq.end())
				return 0;
			return 1;
		}else if(it2==seq.end())
			return -1;
		if((*it1)->second.size()==(*it2)->second.size()){
			for(std::multiset<STrans>::const_iterator itt1=(*it1)->second.begin(),itt2=(*it2)->second.begin();itt1!=(*it1)->second.end();itt1++,itt2++){
				int test;
				if(itt2->m_Name!=itt1->m_Name){//Attention, les transitions modifiées n'ont pas la même adresse de nom de classe
					test = strcmp(itt2->m_Name,itt1->m_Name);
					if(test)
						return test;
				}
				test = itt2->m_OutInd-itt1->m_OutInd;
				if(test!=0)
					return test;
			}
		}else if((*it2)->second.size()>(*it1)->second.size())
			return 1;
		else
			return -1;
	}
}

void CRedSeq::add(CRedSeq const&seq){
	std::vector<SRedBranch*>::const_iterator it2=seq.begin();
	for(std::vector<SRedBranch*>::iterator it1=begin();it1!=end();it1++,it2++)
		(*it1)->first+=(*it2)->first;
	m_Nb+=seq.m_Nb;
}

bool CRedSeq::operator<(CRedSeq const&seq)const{
	return compare(seq)>0;
}

float CAnalyser::m_ConditionLimit;

CAnalyser::CAnalyser(CSystem*system):m_System(*(system?system:new CSystem("Analyse"))),m_LocalSystem(system==NULL),m_SeqFilter(NULL),m_TrClasses(*new CTrClasses),m_SeqClasses(*new CSeqClasses)
{
}

CAnalyser::~CAnalyser(void)
{
	delete&m_TrClasses;
	delete&m_SeqClasses;
	if(m_LocalSystem)
		delete &m_System;
	delete m_SeqFilter;
}

void CAnalyser::loadResults(char const*resFile)
{
	m_System.loadResults(resFile);
}

std::vector<IMonitored const*>CAnalyser::monitoredElts(char const*type)const{
	std::vector<IMonitored const*>res;
	std::vector<IMonitored const*>vM=m_System.getMonitoredElts();
	for(std::vector<IMonitored const*>::const_iterator it=vM.begin();it!=vM.end();it++)
		if(!type || !strcmp((*it)->type(),type))
			res.push_back(*it);
	return res;
}

IMonitored const*CAnalyser::monitoredElt(char const*name,char const*type)const{
	return m_System.getMonitoredElt(name,type);
}

std::vector<IIndicator*>CAnalyser::indicators()const{
	return m_System.getIndicators();
}

IIndicator const*CAnalyser::indicator(char const*name)const{
	return m_System.indicator(name);
}

std::vector<double>CAnalyser::instants()const{
	return m_System.instants();
}

std::vector<ISequence*>CAnalyser::filteredSeq()const{
	std::vector<ISequence*>vRes;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its))
			vRes.push_back(*its);
	return vRes;
}

bool CAnalyser::keepFilteredSeq(bool restart){
	//Vérification
	std::vector<ITransition*>vT=m_System.getTransitions();
	for(std::vector<ITransition*>::iterator it=vT.begin();it!=vT.end();it++)
		if(!(*it)->monitor())
			ILogManager::glLogManager().throwError(formatMsg("Il n'est pas possible de prolonger les séquences car la transition %s n'a pas été monitorée",(*it)->name()).c_str());
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its))
			(*its)->setToKeep(restart);
	return true;
}

void CAnalyser::allSeqValues(ISequence const*seq,char const*name,std::vector<double>&ts,std::vector<float>&values)const{
	ILogManager::CCtx ctx("Pendant la récupération de toutes les valeurs de la séquence");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	ts.clear();
	values.clear();
	std::map<IMonitored const*,CHoldAny>::const_iterator itd;
	for(CSequence::CIter its(*static_cast<CSequence const*>(seq));!its.atEnd();its.next())
		if((itd=its.data().find(obj))!=its.data().end()){
			ts.push_back(its.time().m_Time);
			values.push_back(convertHA<float>(itd->second));
		}
}

std::vector<float>CAnalyser::allValues(char const*name,double t)const
{
	ILogManager::CCtx ctx("Pendant le calcul de toutes les valeurs");
	if(t<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	std::vector<float>values;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its))
			values.push_back((*its)->value(obj,t));
	return values;
}

std::vector<SEQ_ID>CAnalyser::occurrences(char const*name,std::vector<double>const&ts,ICondFctFloat const&condition)const
{
	ILogManager::CCtx ctx("Pendant le calcul des occurrences");
	if(ts.size()==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<ts.size();i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	std::vector<SEQ_ID>nbs(ts.size(),0);
	std::vector<float>values;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			values.clear();
			(*its)->values(obj,ts,values);
			for(std::vector<float>::const_iterator itv=values.begin();itv!=values.end();itv++)
				if(condition(*itv))
					nbs[itv-values.begin()]++;
		}
	return nbs;
}

std::vector<float>CAnalyser::realized(char const*name,std::vector<double>const&ts,ICondFctFloat const&condition)const
{
	ILogManager::CCtx ctx("Pendant le calcul des probabilités de réalisation");
	if(ts.size()==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<ts.size();i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	std::vector<SEQ_ID>nbs(ts.size(),0);
	SEQ_ID nbS=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			std::vector<bool> breal;
			(*its)->realized(obj,condition,ts,breal);
			for(unsigned i=0;i<ts.size();i++)
				nbs[i]+=breal[i]?1:0;
			nbS++;
		}
	std::vector<float>pr(ts.size(),0);
	for(unsigned i=0;i<nbs.size();i++)
		pr[i]=(float)(nbs[i]/(double)nbS);
	return pr;
}

float CAnalyser::residenceTime(char const*name,double it,double ft,ICondFctFloat const&condition)const
{
	ILogManager::CCtx ctx("Pendant le calcul du temps de résidence");
	if(it<0)
		ILogManager::glLogManager().throwError("Instant de début négatif");
	if(it>=ft)
		ILogManager::glLogManager().throwError("Instant de début supérieur à l'instant de fin de calcul");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	double time=0;
	SEQ_ID nb=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			time+=(*its)->residenceTime(obj,it,ft,condition);
			nb++;
		}
	time/=nb;
	return (float)time;
}

std::vector<PyTime>CAnalyser::residenceTimes(char const*name,double it,double ft,std::vector<float>const&lims)const{
	ILogManager::CCtx ctx("Pendant le calcul des temps de résidence");
	if(it>=ft)
		ILogManager::glLogManager().throwError("Instant de début supérieur à l'instant de fin de calcul");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	SEQ_ID nb=0;
	std::vector<PyTime>resTs(lims.size()+1,0);
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			(*its)->residenceTimes(obj,it,ft,lims,resTs);
		}
	for(std::vector<float>::size_type i=0;i<resTs.size();i++)
		resTs[i]/=nb;
	return resTs;
}

std::vector<float>CAnalyser::meanDistribution(char const * name, std::vector<double>const & ts, std::vector<float>const & lims)const
{
	ILogManager::CCtx ctx("Pendant le calcul de la distribution moyenne");
	if(lims.size()==0)
		ILogManager::glLogManager().throwError("Aucun échantillon de calcul");
	for(size_t i=1;i<lims.size();i++)
		if(lims[i-1]>=lims[i])
			ILogManager::glLogManager().throwError("La suite d'échantillons n'est pas croissante");
		
	//Recherche de l'élément monitoré correspondant
	IMonitored const * obj = m_System.getMonitoredElt(name);
	SEQ_ID nb = 0;
	std::vector<float>dist(lims.size()+1);
	std::vector<SEQ_ID>nbs(dist.size(),0);
	
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			std::vector<float>values;
			(*its)->values(obj,ts,values);
			nb++;
			for(auto it=values.begin();it!=values.end();it++){
				std::vector<float>::const_iterator itl=std::upper_bound(lims.begin(),lims.end(),*it);
				if(itl==lims.end())
					nbs.back()++;
				else
					nbs[itl-lims.begin()+(*itl<=*it)]++;
			}
		}
	for(size_t i=0;i<=lims.size();i++)
		dist[i] = (float)(nbs[i]/(double)(nb*ts.size()));
	
	return dist;
}

std::vector<float>CAnalyser::distribution(char const*name,double it,std::vector<float>const&lims)const{
	ILogManager::CCtx ctx("Pendant le calcul de la distribution");
	if(it<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	if(lims.size()==0)
		ILogManager::glLogManager().throwError("Aucun échantillon de calcul");
	for(size_t i=1;i<lims.size();i++)
		if(lims[i-1]>=lims[i])
			ILogManager::glLogManager().throwError("La suite d'échantillons n'est pas croissante");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	SEQ_ID nb=0;
	std::vector<float>dist(lims.size()+1);
	std::vector<SEQ_ID>nbs(dist.size(),0);
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			float val=(*its)->value(obj,it);
			std::vector<float>::const_iterator itl=std::upper_bound(lims.begin(),lims.end(),val);
			if(itl==lims.end())
				nbs.back()++;
			else
				nbs[itl-lims.begin()+(*itl<=val)]++;
		}
	for(size_t i=0;i<=lims.size();i++)
		dist[i]=(float)(nbs[i]/(double)nb);
	return dist;
}

void CAnalyser::extremeValues(char const*name,float&min,float&max)const
{
	ILogManager::CCtx ctx("Pendant le calcul des valeurs extrêmes");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	min=max=convertHA<float>(m_System.systemP().getMonitoredInit(obj));
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its))
			(*its)->extremeValues(obj,min,max);
}

float CAnalyser::quantileGt(char const*name,float t,float pct)const{
	std::set<float>vs;
	ILogManager::CCtx ctx("Pendant le calcul du quantile superieur");
	if(pct<0 || pct>100)
		ILogManager::glLogManager().throwError("Pourcentage hors limite");
	IMonitored const*obj=m_System.getMonitoredElt(name);
	SEQ_ID nb=(SEQ_ID)(m_System.nbSequences()*pct/100),nbTot=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			vs.insert((*its)->value(obj,t));
			nbTot++;
		}
		if(vs.size()>nb+1)
			vs.erase(vs.begin());
	}
	nb=(SEQ_ID)(nbTot*pct/100);
	while(vs.size()>nb+1)
		vs.erase(vs.begin());
	return *vs.rbegin();
}

float CAnalyser::quantileLe(char const*name,float t,float pct)const{
	std::set<float>vs;
	ILogManager::CCtx ctx("Pendant le calcul du quantile inferieur");
	if(pct<0 || pct>100)
		ILogManager::glLogManager().throwError("Pourcentage hors limite");
	IMonitored const*obj=m_System.getMonitoredElt(name);
	SEQ_ID nb=(SEQ_ID)(m_System.nbSequences()*pct/100),nbTot=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			vs.insert((*its)->value(obj,t));
			nbTot++;
		}
		if(vs.size()>nb+1)
			vs.erase(--vs.end());
	}
	nb=(SEQ_ID)(nbTot*pct/100);
	while(vs.size()>nb+1)
		vs.erase(--vs.end());
	return *vs.begin();
}

float CAnalyser::quantile(char const*name,float pct,float precis)const
{
	ILogManager::CCtx ctx("Pendant le calcul du quantile");
	if(pct<0 || pct>100)
		ILogManager::glLogManager().throwError("Pourcentage hors limite");
	if(precis<=0)
		ILogManager::glLogManager().throwError("Précision négative");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	if(obj){
		float min,max,tps,tpsSeq=(float)m_System.systemP().sequences()[0]->endTime();
		extremeValues(name,min,max);
		while((max-min)>precis){
			m_ConditionLimit=(min+max)/2;
			tps=residenceTime(name,0,HUGE_VAL,condition);
			if(tps/tpsSeq*100>pct)
				min=m_ConditionLimit;
			else
				max=m_ConditionLimit;
		}
		return (min+max)/2;
	}
	return 0;
}


void CAnalyser::quantiles(char const*name, std::vector<double>const & ts, std::vector<float> & fracilesMin, float pctMin, std::vector<float> & fracilesMax, float pctMax)const
{
	ILogManager::CCtx ctx("Pendant le calcul des quatiles inf et sup");

	if(pctMin < 0 || pctMin > 100)
		ILogManager::glLogManager().throwError("Pourcentage hors limite du quantile Inf");
	if(pctMax < 0 || pctMax > 100)
		ILogManager::glLogManager().throwError("Pourcentage hors limite du quantile Sup");

	size_t								nbT   = ts.size();
	std::vector<std::vector<float> >	lValues;
	std::vector<float>					values;

	lValues.resize(nbT);
	fracilesMin.resize(nbT);
	fracilesMax.resize(nbT);

	//Recherche de l'élément monitoré correspondant
	IMonitored const*		obj  = m_System.getMonitoredElt(name);

	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its))
		{
			values.clear();
			(*its)->values(obj, ts, values);
			for(size_t i = 0; i< nbT; i++)
				lValues[i].push_back(values[i]);
		}
	SEQ_ID	nbMin = (SEQ_ID)(m_System.nbSequences()*pctMin/100.);
	SEQ_ID	nbMax = (SEQ_ID)(m_System.nbSequences()*(1.-pctMax/100.));
	for(size_t i = 0; i< nbT; i++)
	{
		std::sort(lValues[i].begin(), lValues[i].end());
		fracilesMin[i]=lValues[i][nbMin];
		fracilesMax[i]=lValues[i][nbMax];
	}
}

std::vector<float> CAnalyser::quantilesGt(char const*name, std::vector<double>const & ts, float pct)const
{
	std::vector<float>res,temp;
	quantiles(name,ts,res,pct,temp,0);
	return res;
}

std::vector<float> CAnalyser::quantilesLe(char const*name, std::vector<double>const & ts, float pct)const
{
	std::vector<float>res,temp;
	quantiles(name,ts,temp,0,res,pct);
	return res;
}

float CAnalyser::meanValue(char const*name,double it,double ft)const
{
	ILogManager::CCtx ctx("Pendant le calcul de la moyenne sur une durée");
	if(it>=ft)
		ILogManager::glLogManager().throwError("Instant de début supérieur à l'instant de fin de calcul");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	PyTime lit(it),lft(ft);
	double mean=0;
	SEQ_ID nb=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			mean+=(*its)->integralValue(obj,lit,lft);
			nb++;
		}
	mean/=nb*(lft-lit);
	return (float)mean;
}

void CAnalyser::meanValue(char const*name,double t,float&mean,float&stdDev)const
{
	ILogManager::CCtx ctx("Pendant le calcul de la moyenne à un instant");
	if(t)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	PyTime tm(t);
	double vMoy=0,vVar=0,lVal;
	SEQ_ID nb=0;
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			vMoy+=lVal=(*its)->value(obj,tm);
			vVar+=lVal*lVal;
		}
	vMoy/=nb;
	vVar=(vVar-vMoy*vMoy*nb)/nb;
	mean=(float)vMoy;
	stdDev=(float)vVar;
}

void CAnalyser::meanValues(char const*name,std::vector<double>const&ts,std::vector<float>&means,std::vector<float>&stdDevs)const
{
	ILogManager::CCtx ctx("Pendant le calcul de moyennes");
	size_t nbT=ts.size();
	if(nbT==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<nbT;i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	unsigned long long nb=0;
	std::vector<PyTime>					tms;
	for(size_t i=0;i<nbT;i++)
		tms.push_back(PyTime(ts[i]));

	std::vector<double>	vM(nbT,0),vV(nbT,0);
	std::vector<float>lVals(nbT);

	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			lVals.clear();
			(*its)->values(obj, tms,lVals);
		}
		for(size_t i=0;i<nbT;i++){
			vM[i]+=lVals[i];
			vV[i]+=lVals[i]*lVals[i];
		}
	}
	stdDevs.resize(nbT);
	means.resize(nbT);
	for(size_t i=0;i<nbT;i++){
		means[i]=(float)vM[i]/nb;
		stdDevs[i]=(float)std::sqrt(std::abs((vV[i]-vM[i]*vM[i]/nb)/nb));
	}
}

std::vector<float>CAnalyser::meanValues(char const*name,std::vector<double>const&ts)const
{
	ILogManager::CCtx ctx("Pendant le calcul de moyennes");
	size_t nbT=ts.size();
	if(nbT==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<nbT;i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	unsigned long long nb=0;
	std::vector<PyTime>					tms;
	for(size_t i=0;i<nbT;i++)
		tms.push_back(PyTime(ts[i]));

	std::vector<double>	vM(nbT,0);
	std::vector<float>lVals(nbT);

	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			lVals.clear();
			(*its)->values(obj, tms,lVals);
		}
		for(size_t i=0;i<nbT;i++)
			vM[i]+=lVals[i];
	}
	std::vector<float>means(nbT);
	for(size_t i=0;i<nbT;i++)
		means[i]=(float)vM[i]/nb;
	return means;
}

std::vector<float>CAnalyser::stdDevValues(char const*name,std::vector<double>const&ts)const
{
	ILogManager::CCtx ctx("Pendant le calcul de d'écarts types");
	size_t nbT=ts.size();
	if(nbT==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<nbT;i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	unsigned long long nb=0;
	std::vector<PyTime>					tms;
	for(size_t i=0;i<nbT;i++)
		tms.push_back(PyTime(ts[i]));

	std::vector<double>	vM(nbT,0),vV(nbT,0);
	std::vector<float>lVals(nbT);

	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			lVals.clear();
			(*its)->values(obj, tms,lVals);
		}
		for(size_t i=0;i<nbT;i++){
			vM[i]+=lVals[i];
			vV[i]+=lVals[i]*lVals[i];
		}
	}
	std::vector<float>stdDevs(nbT);
	for(size_t i=0;i<nbT;i++)
		stdDevs[i]=(float)std::sqrt(std::abs((vV[i]-vM[i]*vM[i]/nb)/nb));
	return stdDevs;
}

std::vector<float>CAnalyser::confIntValues(char const*name,std::vector<double>const&ts,float pct)const{
	ILogManager::CCtx ctx("Pendant le calcul d'intervalles de confiance");
	size_t nbT=ts.size();
	if(nbT==0)
		ILogManager::glLogManager().throwError("Aucun instant de calcul");
	if(ts[0]<0)
		ILogManager::glLogManager().throwError("Instant de calcul négatif");
	for(size_t i=1;i<nbT;i++)
		if(ts[i-1]>=ts[i])
			ILogManager::glLogManager().throwError("La suite d'instants n'est pas croissante");
	
	//Recherche de l'élément monitoré correspondant
	IMonitored const*obj=m_System.getMonitoredElt(name);
	unsigned long long nb=0;
	std::vector<PyTime>					tms;
	for(size_t i=0;i<nbT;i++)
		tms.push_back(PyTime(ts[i]));

	std::vector<double>	vM(nbT,0),vV(nbT,0);
	std::vector<float>lVals(nbT);

	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++){
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nb++;
			lVals.clear();
			(*its)->values(obj, tms,lVals);
		}
		for(size_t i=0;i<nbT;i++){
			vM[i]+=lVals[i];
			vV[i]+=lVals[i]*lVals[i];
		}
	}
	std::vector<float>confInt(nbT);
	//On utilise sqrt(2)*erf-1(pct/100)*EcartType/sqrt(n)
	float coef=trng::math::inv_erf(pct/100)*sqrt(2.F)/sqrt((float)nb);
	for(size_t i=0;i<nbT;i++)
		confInt[i]=coef*(float)std::sqrt(std::abs((vV[i]-vM[i]*vM[i]/nb)/nb));
	return confInt;
}

void CAnalyser::setSeqFilter(IFilterFct*seqFilter){
	delete m_SeqFilter;
	m_SeqFilter=seqFilter;
}

void CAnalyser::setSeqFilter(bool (*seqFilter)(ISequence const&)){
	delete m_SeqFilter;
	m_SeqFilter=new CFilterFct(seqFilter);
}

bool CAnalyser::printFilteredSeq(float pct,char const*file,char const*xsltFile)const{
	ILogManager::CCtx ctx("Pendant la restitution des séquences réduites");
	if(pct<0 || pct>100)
		return 0==ILogManager::glLogManager().msgError("Pourcentage hors limites");
	//On construit les séquences réduites filtrées
	std::map<std::string,std::map<CRedSeq const*,SEQ_ID> >mSeqCl;
	for(auto it=m_SeqClasses.begin();it!=m_SeqClasses.end();it++)
		mSeqCl[it->first];
	std::set<CRedSeq>sRS;
	SEQ_ID nbS=0;//Nombre de séquences prises en compte
	for(std::vector<CSequence*>::const_iterator its=m_System.systemP().sequences().begin();its!=m_System.systemP().sequences().end();its++)
		if(m_SeqFilter==NULL || (*m_SeqFilter)(**its)){
			nbS++;
			CRedSeq rs(**its);
			std::pair<std::set<CRedSeq>::iterator,bool>res=sRS.insert(rs);
			if(!res.second)
				const_cast<CRedSeq*>(&*res.first)->add(rs);
			for(auto itc = m_SeqClasses.begin();itc!=m_SeqClasses.end();itc++)
				if((*itc->second)(**its)){//Le filtre est ok pour la séquence, on l'ajoute dans la classe
					std::map<CRedSeq const*,SEQ_ID>&sc=mSeqCl[itc->first];
					std::map<CRedSeq const*,SEQ_ID>::iterator its = sc.find(&*res.first);
					if(its==sc.end())
						sc[&*res.first]=1;
					else
						its->second++;
				}
		}
	//Tri en fonction du nombre de séquences par séquence réduite
	std::set<CRedSeq const*,bool(*)(CRedSeq const*,CRedSeq const*)>sOS(&CRedSeq::nbCompare);
	for(auto it=sRS.begin();it!=sRS.end();it++)
		sOS.insert(&*it);
	if(sOS.size()!=sRS.size())
		return 0==ILogManager::glLogManager().msgError("Erreur1 de gestion des séquences réduites");
	//Attribution de l'index
	//Attention aux indices décroissants liés à l'opérateur de comparaison !!!!
	//-------------------------------------------------------------------------
	SEQ_ID id=sOS.size();
	for(auto it=sOS.begin();it!=sOS.end();it++)
		const_cast<CRedSeq*>(*it)->setId(id--);//Il faudrait reclasser, mais c'est déjà dans le bon ordre
	std::map<CRedSeq,std::set<CRedSeq const*,bool(*)(CRedSeq const*,CRedSeq const*)> >mSeqTrCl;
	if(!m_TrClasses.empty()){//Construction des classes de séquences par transitions
		for(auto it=sRS.begin();it!=sRS.end();it++){
			CRedSeq rs(*it,m_TrClasses);
			if(rs.compare(*it)==0)//Si la séquence n'est pas modifiée, elle ne peut pas entrer dans une classe
				continue;
			auto vSeq=mSeqTrCl.insert(std::pair<CRedSeq,std::set<CRedSeq const*,bool(*)(CRedSeq const*,CRedSeq const*)> >(rs,std::set<CRedSeq const*,bool(*)(CRedSeq const*,CRedSeq const*)>(&CRedSeq::nbCompare)));
			const_cast<CRedSeq*>(&vSeq.first->first)->setNb(vSeq.second?it->nb():(vSeq.first->first.nb()+it->nb()));
			vSeq.first->second.insert(&*it);
			if(1!=sOS.erase(&*it))//Attention au erase qui n'utilise que l'opérateur de comparaison nbCompare : ce dernier doit distinguer chaque élément pour ne pas effacer le mauvais
				return 0==ILogManager::glLogManager().msgError("Erreur2 de gestion des séquences réduites");
		}
	}
	//Tri global des classes
	for(auto it=mSeqTrCl.begin();it!=mSeqTrCl.end();it++)
		sOS.insert(&it->first);
	//Construction du document XML
	CXMLDocument doc;
	CXMLNode&root=*doc.new_root("PYCRS");
	root.add_attr("NS",m_System.nbSequences());
	if(!m_TrClasses.empty()){//Description des classes de transitions
		//Regrouppement des classes
		std::map<std::string,std::vector<std::pair<CTransition const*,int> > >mOC;
		for(auto it=m_TrClasses.begin();it!=m_TrClasses.end();it++)
			mOC[it->second].push_back(it->first);
		for(auto it=mOC.begin();it!=mOC.end();it++){
			CXMLNode&nd=root.add_son("TR_CL");
			nd.add_attr("NAME",it->first.c_str());
			for(auto itt=it->second.begin();itt!=it->second.end();itt++){
				CXMLNode&ndt = nd.add_son("TR");
				ndt.add_attr("NAME",itt->first->name());
				ndt.add_attr("IND",itt->second);
			}
		}
	}
	if(!m_SeqClasses.empty()){//Description et contenu des classes de séquences
		//Tri des classes
		std::multimap<SEQ_ID,std::string>mOSC;
		for(auto it=mSeqCl.begin();it!=mSeqCl.end();it++){
			SEQ_ID nb=0;
			for(auto its=it->second.begin();its!=it->second.end();its++)
				nb+=its->second;
			mOSC.insert(std::pair<SEQ_ID,std::string>(nb,it->first));
		}
		for(auto it=mOSC.rbegin();it!=mOSC.rend();it++){
			CXMLNode&nd=root.add_son("SEQ_CL");
			nd.add_attr("NAME",it->second.c_str());
			nd.add_attr("N",it->first);
			IFilterFct*iFilter=m_SeqClasses.find(it->second)->second;
			if(dynamic_cast<CFilterCompFct const*>(iFilter)){
				CFilterCompFct const*filter=static_cast<CFilterCompFct const*>(iFilter);
				nd.add_attr("OBJ",filter->m_Obj->name());
				nd.add_attr("TYPE",filter->m_Obj->type());
				nd.add_attr("VAL",filter->value());
				nd.add_attr("TIME",filter->m_Time);
				nd.add_attr("OP",filter->opStr());
			}else if(dynamic_cast<CFilterPattern const*>(iFilter))
				nd.add_attr("PAT",static_cast<CFilterPattern const*>(iFilter)->pattern());
			//Tri des séquences de la classe (attention ! le nombre d'emploi de chaque seq red est différent du nom interne de la seq red)
			auto mS = mSeqCl[it->second];
			std::multimap<std::pair<SEQ_ID,SEQ_ID>,CRedSeq const*>mOS;
			for(auto its=mS.begin();its!=mS.end();its++)
				mOS.insert(std::pair<std::pair<SEQ_ID,SEQ_ID>,CRedSeq const*>(std::pair<SEQ_ID,SEQ_ID>(its->second,nbS-its->first->id()),its->first));
			for(auto its=mOS.rbegin();its!=mOS.rend();its++){
				CXMLNode&nds=nd.add_son("SEQ");
				nds.add_attr("ID",its->second->id());
				nds.add_attr("N",its->first.first);
			}
		}
	}
	SEQ_ID nb=0,nbM=(SEQ_ID)(nbS*(pct/100));
	for(auto itg=sOS.rbegin();itg!=sOS.rend() && nb<=nbM;itg++){
		auto itcl=mSeqTrCl.find(**itg);
		nb+=(*itg)->nb();
		if(itcl==mSeqTrCl.end()){//C'est une séquence non modifiée par les classes de transitions
			CXMLNode&nSeq=root.add_son("SEQ");
			(*itg)->dump(nSeq,!m_SeqClasses.empty());
		}else if(itcl->second.size()==1){//C'est une séquence isolée dans sa classe, on la restitue telle quelle
			CXMLNode&nSeq=root.add_son("SEQ");
			(*itcl->second.begin())->dump(nSeq,!m_SeqClasses.empty());
		}else{//C'est une classe de séquences
			CXMLNode&nSeqm=root.add_son("SEQ");
			(*itg)->dump(nSeqm,false);
			for(auto its=itcl->second.rbegin();its!=itcl->second.rend();its++){
				CXMLNode&nSeq=nSeqm.add_son("SEQ");
				(*its)->dump(nSeq,!m_SeqClasses.empty());
			}
		}
	}
	root.add_attr("NSF",nbS);
	std::ofstream ofs(file);
	if(!ofs.is_open())
		return 0==ILogManager::glLogManager().msgError(formatMsg("Le fichier %s ne peut pas être ouvert",file).c_str());
	doc.print_in_stream(ofs);
	if(xsltFile && *xsltFile){//On transforme en html
		std::string htmlFile;
		char const*saxon=CExecute::find_path("saxon9he.jar");
		if(!saxon)
			return 0==ILogManager::glLogManager().msgError("saxon9he.jar est introuvable dans le path");
		char const*xslt=CExecute::find_path(xsltFile);
		if(!xslt){
			delete saxon;
			return 0==ILogManager::glLogManager().msgError(formatMsg("Le fichier %s est introuvable dans le path",xsltFile).c_str());
		}
		if(exists(xslt)<2){
			delete saxon;
			delete xslt;
			return 0==ILogManager::glLogManager().msgError(formatMsg("Le fichier %s ne peut pas être lu",xsltFile).c_str());
		}
		htmlFile=file;
		size_t ext=htmlFile.find_last_of('.');
		if(ext!=std::string::npos)
			htmlFile.resize(ext);
		htmlFile.append(".html");
		#ifdef WIN32
		CExecute exe("java.exe");
		#else
		CExecute exe("java");
		#endif
		std::string cmd=formatMsg("-jar \"%s\" \"-s:%s\" \"-xsl:%s\" \"-o:%s\"",saxon,file,xslt,htmlFile.c_str());
		bool ok=0==exe.execute(cmd.c_str(),-2);
		delete saxon;
		delete xslt;
		return ok;
	}
	return true;
}

bool CAnalyser::analyse(char const*filename){
	ILogManager::CCtx ctx("Pendant l'analyse");
	CXMLParser ps;
	if(!ps.build_xml_tree(filename) || !ps.get_root_node())
		return false;
	CXMLNodeIterSons it(*ps.get_root_node());
	std::ofstream ofs;
	char const*anal_file=ps.get_root_node()->find_attr("FILE")?ps.get_root_node()->find_attr_val("FILE"):NULL;
	if(anal_file){
		ofs.open(anal_file);
		if(!ofs.is_open())
			return 0==ILogManager::glLogManager().msgError(formatMsg("Le fichier %s ne peut pas être ouvert",anal_file).c_str());
	}
	while(it.next()){
		if(m_System.trace() && !ILogManager::glLogManager().traceInhibited())
			ILogManager::trStream()<<"Analyse : "<<it.cur()->get_str_name()<<std::endl;
		if(!strcmp(it.cur()->get_str_name(),"FILTER")){
			if(it.cur()->find_attr("OBJ")){
				CFilterCompFct*lf=new CFilterCompFct();
				lf->m_Obj=monitoredElt(it.cur()->find_attr_val("OBJ",NULL),it.cur()->find_attr_val("TYPE"));
				lf->setValue((float)atof(it.cur()->find_attr_val("VAL",NULL)));
				lf->m_Time=atof(it.cur()->find_attr_val("TIME",NULL));
				if(!lf->setOp(it.cur()->find_attr_val("OP",NULL)))
					return false;
				const_cast<CAnalyser*>(this)->setSeqFilter(lf);
			}else if(it.cur()->find_attr("PAT")){
				try{
					CFilterPattern*fp=new CFilterPattern(it.cur()->find_attr_val("PAT"));
					const_cast<CAnalyser*>(this)->setSeqFilter(fp);
				}catch(...){
					return false;
				}
			}else
				const_cast<CAnalyser*>(this)->setSeqFilter((IFilterFct*)NULL);
		}else if(!strcmp(it.cur()->get_str_name(),"XML")){
			setSequenceClass(NULL,NULL);
			setTransitionClass(NULL,0,NULL);
			CXMLNodeIterSons itc(*it.cur());
			try{
				while(itc.next())
					if(!strcmp(itc.cur()->get_str_name(),"CLASS")){
						if(itc.cur()->find_attr("TR"))
							setTransitionClass(dynamic_cast<ITransition const*>(monitoredElt(itc.cur()->find_attr_val("TR"),"TR")),atoi(itc.cur()->find_attr_val("IND","0")),itc.cur()->find_attr_val("NAME",NULL));
						else if(itc.cur()->find_attr("OBJ"))
							setSequenceClass(itc.cur()->find_attr_val("NAME",NULL),IFilterFct::newConditionFilter(itc.cur()->find_attr_val("OBJ"),atof(itc.cur()->find_attr_val("TIME")),itc.cur()->find_attr_val("OP"),(float)atof(itc.cur()->find_attr_val("VAL"))));
						else
							setSequenceClass(itc.cur()->find_attr_val("NAME",NULL),IFilterFct::newPatternFilter(itc.cur()->find_attr_val("PAT")));
					}
			}catch(...){
				return false;
			}
			printFilteredSeq((float)atof(it.cur()->find_attr_val("PCT","100")),it.cur()->find_attr_val("FILE",NULL),it.cur()->find_attr_val("XSLT"));
		}else if(!strcmp(it.cur()->get_str_name(),"ALLSEQVS")){
			std::vector<double>ts;
			std::vector<float>vv;
			ISequence const*seq=m_System.sequence(atoi(it.cur()->find_attr_val("IND")));
			if(!seq)
				return 0==ILogManager::glLogManager().msgError(formatMsg("La %sème séquence n'existe pas",it.cur()->find_attr_val("IND")).c_str());
			allSeqValues(seq,it.cur()->find_attr_val("OBJ",NULL),ts,vv);
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"Toutes valeurs de la séquence = "<<it.cur()->find_attr_val("IND",NULL)<<"\nInstant\tValeur\n";
			for(size_t i=0;i<vv.size();i++)
				ofs<<ts[i]<<'\t'<<vv[i]<<"\n";
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"ALLVS")){
			std::vector<float>vv=allValues(it.cur()->find_attr_val("OBJ",NULL),atof(it.cur()->find_attr_val("TIME",NULL)));
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"Toutes valeurs à T="<<it.cur()->find_attr_val("TIME",NULL);
			for(size_t i=0;i<vv.size();i++)
				ofs<<'\t'<<vv[i];
			ofs<<'\n';
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"MEAN")){//Moyennes, écarts types et intervalles de confiance
			std::vector<PyTime>vt;
			char const*times=it.cur()->find_attr_val("ECHTS");
			if(times && *times){
				double tm,tM;
				int nb;
				if(sscanf(times,"%lf;%lf;%d",&tm,&tM,&nb)!=3 || nb<=1 || tm<0 || tM<=tm)
					return 0==ILogManager::glLogManager().msgError((std::string("Echantillonnage mal défini : ")+times).c_str());
				for(int i=0;i<=nb;i++)
					vt.push_back(tm+i*(tM-tm)/nb);
			}else
				for(times=it.cur()->find_attr_val("TIMES",NULL);times;times=strchr(times,';')){
					if(times[0]==';')
						times++;
					double t;
					sscanf(times,"%lf",&t);
					vt.push_back(t);
				}
			std::vector<float>means,stdDevs,confInts;
			meanValues(it.cur()->find_attr_val("OBJ",NULL),vt,means,stdDevs);
			if(it.cur()->find_attr("PCT"))
				confInts = confIntValues(it.cur()->find_attr_val("OBJ",NULL),vt,(float)atof(it.cur()->find_attr_val("PCT")));
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"T\tmoyenne\técart type"<<(confInts.empty()?"\n":"\tint conf\n");
			for(size_t i=0;i<means.size();i++){
				ofs<<vt[i]<<'\t'<<means[i]<<'\t'<<stdDevs[i];
				if(!confInts.empty())
					ofs<<'\t'<<confInts[i];
				ofs<<'\n';
			}
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"FRACT")){//Fractiles
			float pct=(float)atof(it.cur()->find_attr_val("PCT",NULL)),prec=(float)atof(it.cur()->find_attr_val("PREC",NULL));
			(anal_file?ofs:std::cout)<<"Fractile "<<pct<<"% ("<<prec<<") "<<quantile(it.cur()->find_attr_val("OBJ",NULL),pct,prec);
		}else if(!strcmp(it.cur()->get_str_name(),"OCC")){//Nombre d'occurrences
			std::vector<PyTime>vt;
			char const*times=it.cur()->find_attr_val("ECHTS");
			if(times && *times){
				double tm,tM;
				int nb;
				if(sscanf(times,"%lf;%lf;%d",&tm,&tM,&nb)!=3 || nb<=1 || tm<0 || tM<=tm)
					return 0==ILogManager::glLogManager().msgError((std::string("Echantillonnage mal défini : ")+times).c_str());
				for(int i=0;i<=nb;i++)
					vt.push_back(tm+i*(tM-tm)/nb);
			}else
				for(times=it.cur()->find_attr_val("TIMES",NULL);times;times=strchr(times,';')){
					if(times[0]==';')
						times++;
					double t;
					sscanf(times,"%lf",&t);
					vt.push_back(t);
				}
			CCondCompFloat lc;
			lc.setValue((float)atof(it.cur()->find_attr_val("VAL",NULL)));
			if(!lc.setOp(it.cur()->find_attr_val("OP",NULL)))
				return false;
			std::vector<size_t>nbs=occurrences(it.cur()->find_attr_val("OBJ",NULL),vt,lc);
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"T\tnb occ "<<it.cur()->find_attr_val("OBJ",NULL)<<it.cur()->find_attr_val("OP",NULL)<<lc.value()<<"\n";
			for(size_t i=0;i<vt.size();i++)
				ofs<<vt[i]<<'\t'<<nbs[i]<<'\n';
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"REAL")){//Réalisation
			std::vector<PyTime>vt;
			char const*times=it.cur()->find_attr_val("ECHTS");
			if(times && *times){
				double tm,tM;
				int nb;
				if(sscanf(times,"%lf;%lf;%d",&tm,&tM,&nb)!=3 || nb<=1 || tm<0 || tM<=tm)
					return 0==ILogManager::glLogManager().msgError((std::string("Echantillonnage mal défini : ")+times).c_str());
				for(int i=0;i<=nb;i++)
					vt.push_back(tm+i*(tM-tm)/nb);
			}else
				for(times=it.cur()->find_attr_val("TIMES",NULL);times;times=strchr(times,';')){
					if(times[0]==';')
						times++;
					double t;
					sscanf(times,"%lf",&t);
					vt.push_back(t);
				}
			CCondCompFloat lc;
			lc.setValue((float)atof(it.cur()->find_attr_val("VAL",NULL)));
			if(!lc.setOp(it.cur()->find_attr_val("OP",NULL)))
				return false;
			std::vector<float>ps=realized(it.cur()->find_attr_val("OBJ",NULL),vt,lc);
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"T\tp real "<<it.cur()->find_attr_val("OBJ",NULL)<<it.cur()->find_attr_val("OP",NULL)<<lc.value()<<"\n";
			for(size_t i=0;i<vt.size();i++)
				ofs<<vt[i]<<'\t'<<ps[i]<<'\n';
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"DIST")){//Distribution
			std::vector<float>vv;
			char const*vals=it.cur()->find_attr_val("ECHVS");
			if(vals && *vals){
				float vm,vM;
				int nb;
				if(sscanf(vals,"%f;%f;%d",&vm,&vM,&nb)!=3 || nb<=1 || vM<=vm)
					return 0==ILogManager::glLogManager().msgError((std::string("Echantillonnage mal défini : ")+vals).c_str());
				for(int i=0;i<=nb;i++)
					vv.push_back(vm+i*(vM-vm)/nb);
			}else
				for(vals=it.cur()->find_attr_val("VALS",NULL);vals;vals=strchr(vals,';')){
					if(vals[0]==';')
						vals++;
					float v;
					sscanf(vals,"%f",&v);
					vv.push_back(v);
				}
			std::vector<float>ps=distribution(it.cur()->find_attr_val("OBJ",NULL),atof(it.cur()->find_attr_val("TIME",NULL)),vv);
			if(!anal_file)
				ofs.open(it.cur()->find_attr_val("FILE",NULL));
			ofs<<"Segment\tproba "<<it.cur()->find_attr_val("OBJ",NULL)<<" "<<it.cur()->find_attr_val("TIME",NULL)<<"\n";
			for(size_t i=0;i<=vv.size();i++)
				ofs<<((i<vv.size())?"<":">=")<<vv[(i<vv.size())?i:(i-1)]<<'\t'<<ps[i]<<'\n';
			if(!anal_file)
				ofs.close();
		}else if(!strcmp(it.cur()->get_str_name(),"REST")){//Temps de résidence
			CCondCompFloat lc;
			lc.setValue((float)atof(it.cur()->find_attr_val("VAL",NULL)));
			if(!lc.setOp(it.cur()->find_attr_val("OP",NULL)))
				return false;
			double tmin=atof(it.cur()->find_attr_val("TMIN","0")),tmax=HUGE_VAL;
			if(it.cur()->find_attr("TMAX"))
				tmax=atof(it.cur()->find_attr_val("TMAX"));
			(anal_file?ofs:std::cout)<<"Tps de résidence "<<it.cur()->find_attr_val("OBJ",NULL)<<it.cur()->find_attr_val("OP",NULL)<<lc.value()<<" : "<<residenceTime(it.cur()->find_attr_val("OBJ",NULL),tmin,tmax,lc)<<"\n";
		}else if(!strcmp(it.cur()->get_str_name(),"RESTART") || !strcmp(it.cur()->get_str_name(),"KEEP")){
			if(!keepFilteredSeq(0!=atoi(it.cur()->find_attr_val("VAL",NULL))))
				return false;
		}else if(!strcmp(it.cur()->get_str_name(),"KEEP_IND")){
			IIndicator*ind=m_System.indicator(it.cur()->find_attr_val("NAME"));
			if(!ind)
				return 0==ILogManager::glLogManager().msgError("Indicateur introuvable");
			ind->setToKeep(0!=atoi(it.cur()->find_attr_val("VAL",NULL)));
		}else
			return 0==ILogManager::glLogManager().msgError(formatMsg("Balise inconnue %s",it.cur()->get_str_name()).c_str());
	}
	return true;
}

void CAnalyser::setTransitionClass(ITransition const*trans,int ind,char const*className){
	if(!className || !className[0]){
		if(trans)//Suppression de la transition de la classe
			m_TrClasses.erase(std::pair<CTransition const*,int>(static_cast<CTransition const*>(trans),ind));
		else//Suppression de toutes les classes
			m_TrClasses.clear();
	}else if(trans)//Placement de la transition dans la classe
		m_TrClasses[std::pair<CTransition const*,int>(static_cast<CTransition const*>(trans),ind)]=className;
	else//Suppression de la classe
		for(auto it=m_TrClasses.begin();it!=m_TrClasses.end();)
			it = it->second==className?m_TrClasses.erase(it):(++it);
}

void CAnalyser::setSequenceClass(char const*className,IFilterFct*filter){
	if(!className){//Suppression de toutes les classes
		for(auto it = m_SeqClasses.begin();it!=m_SeqClasses.end();it++)
			delete it->second;
		m_SeqClasses.clear();
		return;
	}
	auto it = m_SeqClasses.find(className);
	if(it!=m_SeqClasses.end())
		delete it->second;//Suppression de l'ancien filtre s'il existe
	if(!filter){//Suppression de la classe
		if(it!=m_SeqClasses.end())
			m_SeqClasses.erase(it);
	}else{//Définition de la classe
		m_SeqClasses[className]=filter;
	}
}
