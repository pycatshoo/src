/*************************************
*         Copyright 2015 EDF         *
*************************************/
#include "Automate.h"

#include "State.h"
#include "Transition.h"
#include "Component.h"

CAutomaton::CAutomaton(CNamedComp&parent,char const*name):IVariable(parent,name),m_CurState(NULL),m_OldState(NULL),m_InitState(0),m_Index(-1),m_InitIndex(-1)
{
}

CAutomaton::~CAutomaton(void)
{
}

void CAutomaton::setValueError(){
	ILogManager::glLogManager().throwError(formatMsg("Il est interdit de modifier l'état courant de l'automate %s",name()).c_str());
}

std::string CAutomaton::sValue()const{
	return formatMsg("%d",m_Index);
}

std::string CAutomaton::sInitValue()const{
	return formatMsg("%d",iInitValue());
}

void CAutomaton::printTrace()const{
	ILogManager::trStream()<<name()<<"("<<currentTime()<<") : "<<(m_OldState?m_OldState->basename():"")<<"->"<<(m_CurState?m_CurState->basename():"")<<std::endl;
}

std::vector<IState*>CAutomaton::states()const{
	return(std::vector<IState*>(m_States.begin(),m_States.end()));
}

void CAutomaton::addState(CState*st){
	if(st->m_Automate)
		ILogManager::glLogManager().throwError(formatMsg("L'état %s appartient déjà à l'automate %s",st->name(),st->m_Automate->basename()).c_str());
	for(std::set<CState*,ncLess>::const_iterator it=m_States.begin();it!=m_States.end();it++)
		if((*it)->index()==st->index()){
			ILogManager::glLogManager().msgWarning(formatMsg("Les états %s et %s ont le même indice %d",st->name(),(*it)->name(),st->index()).c_str(),TErrorLevel::warning);
			break;
		}
	if(st->parent()!=parent())
		ILogManager::glLogManager().throwError(formatMsg("L'état %s ne peut appartenir à l'automate %s",st->name(),name()).c_str());
	m_States.insert(st);
	st->m_Automate=this;
}

IState*CAutomaton::addState(char const*name,int index){
	return dynamic_cast<CComponent*>(parent())->addState(basename(),name,index);
}

void CAutomaton::merge(CAutomaton&other){
	if(this==&other)
		return;
	if(basename()[0]!='#' || other.basename()[0]!='#')
		ILogManager::glLogManager().throwError(formatMsg("Il ne peut pas exister de transition entre les automates %s et %s",name(),other.name()).c_str());
	for(std::set<CState*,ncLess>::const_iterator it=other.m_States.begin();it!=other.m_States.end();it++){
		(*it)->m_Automate=NULL;
		(*it)->m_Index=(int)m_States.size();
		addState(*it);
		if(other.initState()==*it)
			setInitState(*it);
	}
	other.m_States.clear();
}

void CAutomaton::setInitIndex(int index){
	if(!m_States.empty())
		ILogManager::glLogManager().throwError(formatMsg("Il est interdit de modifier l'index de l'état initial de l'automate %s",name()).c_str());
	m_InitIndex=index;
}

void CAutomaton::setInitState(IState*st){
	if(st && m_States.find(static_cast<CState*>(st))==m_States.end())
		ILogManager::glLogManager().throwError(formatMsg("L'état %s ne fait pas partie de l'automate %s",st->name(),name()).c_str());
	if(m_InitState && isRunning())
		ILogManager::glLogManager().throwError("L'état initial ne peut pas être modifié en cours de simulation");
	m_CurState=m_OldState=m_InitState=static_cast<CState*>(st);
	m_Index=m_InitIndex=st?st->index():-1;
}

void CAutomaton::setCurrentState(CState*st){
	if(st && m_States.find(st)==m_States.end())
		ILogManager::glLogManager().throwError(formatMsg("L'état %s ne fait pas partie de l'automate %s",st->name(),name()).c_str());
	if(st && m_CurState && st!=m_CurState)
		ILogManager::glLogManager().throwError(formatMsg("L'automate %s a déjà un état courant",name()).c_str());
	else{
		m_OldState=m_CurState;
		m_CurState=st;
		m_Index=st?st->m_Index:-1;
		if(st && isRunning()){
			tellModified(0);
			monitorValue();
		}
	}
}

IState*CAutomaton::currentState()const{
	return m_CurState;
}

IState*CAutomaton::initState()const{
	return m_InitState;
}

void CAutomaton::fill(CState const&st,std::set<CState const*>&stGraph)const{
	if(!stGraph.insert(&st).second)
		return;//Déjà fait
	for(std::vector<ITransition*>::const_iterator it=st.m_Trans.begin();it!=st.m_Trans.end();it++)
		for(size_t i=0;i<(*it)->targetCount();i++)
			if((*it)->getTarget((TGT_ID)i))
				fill(*static_cast<CState const*>((*it)->getTarget((TGT_ID)i)),stGraph);
}

bool CAutomaton::verify(){
	bool ok=true;
	if(m_InitState==NULL)
		ok=TErrorLevel::error!=ILogManager::glLogManager().msgError(formatMsg("L'automate %s n'a pas d'état initial",name()).c_str())&&ok;
	else{
		std::set<CState const*>stGraph;
		fill(*m_InitState,stGraph);
		if(stGraph.size()!=m_States.size()){
			std::string ust;
			for(auto it = m_States.begin();it!=m_States.end();it++)
				if(!stGraph.count(*it))
					ust=ust+", "+(*it)->basename();
			ok=TErrorLevel::error!=ILogManager::glLogManager().msgWarning(formatMsg("Les états %s de l'automate %s ne sont pas accessibles à partir de l'état initial %s",ust.c_str()+2,name(),m_InitState->basename()).c_str(),TErrorLevel::warning)&&ok;
		}
	}
	if(m_States.size()==0)
		ok=TErrorLevel::error!=ILogManager::glLogManager().msgWarning(formatMsg("L'automate %s n'a aucun état",name()).c_str(),TErrorLevel::warning)&&ok;
	if(m_States.size()<=1)
		ok=TErrorLevel::error!=ILogManager::glLogManager().msgWarning(formatMsg("L'automate %s a moins de 2 états",name()).c_str(),TErrorLevel::warning)&&ok;
	return ok;
}
