# 📘 Guide Complet des Outils et Workflows BMad

Bienvenue dans BMad ! Ce guide liste tous les outils (workflows et agents) à votre disposition, classés par module. Pour utiliser un outil, tapez simplement sa commande (ex: `/bmad-bmm-create-prd`) ou appelez l'agent concerné.

---

## 🏗️ Module BMB : Builder (Création sur-mesure)
Outils pour créer et modifier vos propres agents et workflows.

| Outil | Commande | Description |
|---|---|---|
| **Create Agent** | `bmad-agent-builder` | Créer un nouvel agent de toutes pièces. |
| **Edit Agent** | `bmad-agent-builder` | Éditer, convertir ou réparer un agent existant. |
| **Optimize Agent** | `bmad-agent-builder` | Vérifier la qualité et optimiser un agent. |
| **Create Workflow** | `bmad-workflow-builder` | Créer un nouveau workflow ou skill. |
| **Edit/Optimize Workflow** | `bmad-workflow-builder` | Modifier ou valider un workflow existant. |

---

## 🎯 Module BMM : Product & Sprint Management
Outils d'idéation, de planification et d'exécution de développement.

### 💡 Phase 1 : Analyse & Idéation
| Outil | Commande / Agent | Description |
|---|---|---|
| **Brainstorm Project** | `bmad-brainstorming` | Guidage expert pour explorer des idées via diverses techniques. |
| **Market / Domain Research** | `bmad-bmm-market-research` <br> `bmad-bmm-domain-research` | Analyse concurrentielle, de marché et de domaine. |
| **Technical Research** | `bmad-bmm-technical-research` | Faisabilité technique, choix d'architecture. |
| **Create Brief** | `bmad-bmm-create-product-brief` | Documenter et cadrer votre idée de produit. |

### 📝 Phase 2 : Planification (PRD & UX)
| Outil | Commande / Agent | Description |
|---|---|---|
| **Create/Edit/Validate PRD** | `bmad-bmm-create-prd` etc. | Création et édition du cahier des charges (PRD). |
| **Create UX** | `bmad-bmm-create-ux-design` | Définition de l'expérience utilisateur et patterns UX. |

### 🏗️ Phase 3 : Architecture & Solutions
| Outil | Commande / Agent | Description |
|---|---|---|
| **Create Architecture** | `bmad-bmm-create-architecture` | Documenter les décisions techniques. |
| **Create Epics and Stories** | `bmad-bmm-create-epics-and-stories` | Créer la liste des Epics et User Stories. |
| **Check Implementation Readiness**| `bmad-bmm-check-implementation-readiness` | Vérifier l'alignement PRD, UX, Architecture et Stories. |

### 💻 Phase 4 : Implémentation & QA
| Outil | Commande / Agent | Description |
|---|---|---|
| **Sprint Planning & Status** | `bmad-bmm-sprint-planning` <br> `bmad-bmm-sprint-status` | Créer et suivre le plan de sprint. |
| **Create & Dev Story** | `bmad-bmm-create-story` <br> `bmad-bmm-dev-story` | Préparer puis implémenter le code d'une Story. |
| **QA Automation Test** | `bmad-bmm-qa-automate` | Générer des tests automatisés E2E/API. |
| **Code Review** | `bmad-bmm-code-review` | Revue de code automatique experte. |
| **Retrospective** | `bmad-bmm-retrospective` | Revue de fin d'epic ou apprentissages. |

### ⚡ Mode Rapide (Quick Flow) et Outils Continus
| Outil | Commande / Agent | Description |
|---|---|---|
| **Document/Context Project** | `bmad-bmm-document-project` <br> `bmad-bmm-generate-project-context` | Scanner le code pour générer le contexte IA. |
| **Quick Spec & Dev** | `bmad-bmm-quick-spec` <br> `bmad-bmm-quick-dev` | Implémenter des petites tâches sans PRD complexe. <br>`bmad-bmm-quick-dev-new-preview` pour un processus unifié.|
| **Correct Course** | `bmad-bmm-correct-course` | Rectifier le tir en cours de projet. |
| **Write/Validate Document** | Agent : `tech-writer` | Rédaction technique, schémas Mermaid, explications. |

---

## 🧠 Module CIS : Stratégie & Innovation
Outils spécialisés pour l'innovation, la résolution de problèmes et le design thinking.

| Outil | Commande | Description |
|---|---|---|
| **Innovation Strategy** | `bmad-cis-innovation-strategy` | Opportunités de disruption technologique / marché. |
| **Problem Solving** | `bmad-cis-problem-solving` | Méthodologies avancées de résolution de problèmes. |
| **Design Thinking** | `bmad-cis-design-thinking` | Conception centrée sur l'humain. |
| **Storytelling** | `bmad-cis-storytelling` | Cadrages narratifs et communication persuasive. |
| **Brainstorming** | `bmad-cis-brainstorming` | Idéation et réflexion guidée. |

---

## 🛠️ CORE : Outils Universels
Accessibles à tout moment pour la gestion des documents et l'assistance.

| Outil | Commande | Description |
|---|---|---|
| **BMad Help** | `bmad-help` | Ce que vous utilisez en ce moment ! Vous aide à savoir quoi faire. |
| **Party Mode** | `bmad-party-mode` | Lancer des débats entre plusieurs agents (ex: Dev vs architecte). |
| **Editorial Review** | `bmad-editorial-review-prose` <br> `bmad-editorial-review-structure` | Revues de texte (prose) ou de structure documentaire. |
| **Adversarial / Edge Case Review**| `bmad-review-adversarial-general` <br> `bmad-review-edge-case-hunter` | Revue de code impitoyable et traque d'edge cases. |
| **Distillator** | `bmad-distillator` | Compression de texte optimisée pour l'IA. |
| **Index Docs & Shard Document**| `bmad-index-docs` <br> `bmad-shard-doc` | Indexation et découpage de très longs documents. |

---

## 🧪 Module TEA : Tests & Qualité (Test Engineering)
Outils spécialisés en ingénierie de test et qualité logicielle.

| Outil | Commande | Description |
|---|---|---|
| **Teach Me Testing** | `bmad-tea-teach-me-testing` | Apprendre les fondamentaux du test informatique. |
| **Test Design & Framework** | `bmad-tea-testarch-test-design` <br> `bmad-tea-testarch-framework` | Stratégie de test et initialisation de frameworks. |
| **ATDD & Test Automation** | `bmad-tea-testarch-atdd` <br> `bmad-tea-testarch-automate` | Tests d'acceptance et extension de l'automatisation. |
| **CI Setup & Traceability** | `bmad-tea-testarch-ci` <br> `bmad-tea-testarch-trace` | Pipelines CI/CD et matrices de traçabilité. |

---

## 🎨 Module WDS : Whiteboard Design System
Parfaite synergie entre stratégie, design visuel UX/UI et intégration IA.

### 👤 Les Agents WDS (Lanceurs)
- **Wake Saga** (`bmad-wds-saga`) : Stratège. Gère les phases de stratégie et de brief produit.
- **Wake Freya** (`bmad-wds-freya`) : UX Designer. Gère les phases de design interactif et de storyboards.

### 🗺️ Phase 1 : Stratégie & Mapping
| Outil | Commande / Workflow | Description |
|---|---|---|
| **Alignment & Signoff** | `bmad-wds-alignment` | Validation avec les parties prenantes. |
| **Project Brief** | `bmad-wds-project-brief` | Vision produit et critères de succès. |
| **Trigger Mapping** | `bmad-wds-trigger-mapping` | Mapper les objectifs métier sur la psychologie utilisateur. |

### 🖌️ Phase 2 : Design & Expérience
| Outil | Commande / Workflow | Description |
|---|---|---|
| **Outline Scenarios** | `bmad-wds-outline-scenarios` | Définir les parcours utilisateurs (journeys). |
| **Conceptual Sketching / Specs**| `bmad-wds-conceptual-sketching` <br> `bmad-wds-conceptual-specs` | Dessin interactif et spécifications détaillées de chaque écran. |
| **Design System & Visual** | `bmad-wds-design-system` <br> `bmad-wds-visual-design` | Construction du système de composants et prototypes HTML. |
| **Design Delivery** | `bmad-wds-design-delivery` | Empaquetage du design pour les développeurs. |

### 👨‍💻 Phase 3 : Développement & Évolution
| Outil | Commande / Workflow | Description |
|---|---|---|
| **Agentic Development** | `bmad-wds-agentic-development` | L'IA développe de manière itérative depuis les spécifications WDS. |
| **Acceptance Testing** | `bmad-wds-usability-testing` | Tests d'usabilité sur de vrais scénarios. |
| **Product Evolution** | `bmad-wds-product-evolution` | Amélioration continue d'un produit existant. |

---
**💡 Astuce pour démarrer :** 
Si vous lancez un nouveau projet complet en utilisant la méthode standard BMM, le workflow classique est :
`bmad-bmm-create-product-brief` -> `bmad-bmm-create-prd` -> `bmad-bmm-create-architecture` -> `bmad-bmm-create-epics-and-stories` -> `bmad-bmm-sprint-planning`.
Si vous avez un projet existant à documenter, lancez `bmad-bmm-generate-project-context`.
Pour de petites tâches rapides, privilégiez `bmad-bmm-quick-spec` ou `bmad-bmm-quick-dev`.
