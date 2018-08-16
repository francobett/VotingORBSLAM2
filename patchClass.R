rm(list=ls())

library(e1071)

#######################################################################
#######################################################################
patchClassifier <- function(outputSVM, patchPath, dictSize, iter) {

    patchDesc <- read.csv(patchPath,header=FALSE)  
    colnames(patchDesc)<-NULL
    # print(patchDesc)
    # patchDesc <- t(as.numeric(t(unlist(strsplit(patchDesc, split=":")))))
    prefijo   <- paste0("s",dictSize,"_r1_it",iter) 

    ## Lee el modelo SVM desde el disco
    bestSVM <- NULL
    fileBestSVM<- paste0(outputSVM,prefijo,"_csvm_bestmodel.RData")
    if (file.exists(fileBestSVM)){
        ## Carga el objeto 'bestSVM'
        load(fileBestSVM)
        # cat("Modelo SVM", fileBestSVM, " cargado\n")
    } else {
        cat("ERROR al leer modelo", fileBestSVM, "\n")
    }

    ## Clasificación sobre el conjunto de testeo
    ## TODO: VERIFICAR QUE patchDesc ES UNA MATRIZ DE 1 FILA y 128 COLUMNAS
    pred <- predict(c_svm_bestmodel, patchDesc, probability=TRUE)
    prob <- attr(pred, "probabilities")
    print(prob)
    
    ## TODO: DEBERIA SER UN SOLO VALOR!
    prob[2]
}